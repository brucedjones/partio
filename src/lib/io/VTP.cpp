/*
 Copyright (c) 2016 MIT Geonumerics
 Permission is hereby granted, free of charge, to any person obtaining a copy of
 this software and associated documentation files (the "Software"), to deal in
 the Software without restriction, including without limitation the rights to
 use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
 of the Software, and to permit persons to whom the Software is furnished to do
 so, subject to the following conditions:

 The above copyright notice and this permission notice shall be included in all
 copies or substantial portions of the Software.

 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 SOFTWARE.
*/

#include "../Partio.h"
#include "../core/ParticleHeaders.h"
#include "PartioEndian.h"
#include "ZIP.h"
#include "base64.h"
#include "pugixml.hpp"

#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <zlib.h>
#include <vector>
#include <stdlib.h>
#include <math.h>

namespace Partio{

using namespace std;

namespace NumberType {
	enum {UNDEFINED, INT8, UINT8, INT16, UINT16, INT32, UINT32, FLOAT32, FLOAT64};
}
namespace ArrayFormat {
	enum {BINARY, ASCII};
}

struct VTPBlockHeader {
	int blocks;
	int blocksize;
	int last_blocksize;
	int *compressed_blocksizes;
};

struct VTPDataArray {
    char *arrayName;
    int numberOfComponents;
    int size;
    int type;
    void *data;
};

int typeSize(int t)
{
    switch(t)
    {
    case NumberType::FLOAT64:
        return 8;

    case NumberType::FLOAT32:
    case NumberType::INT32:
    case NumberType::UINT32:
        return 4;

    case NumberType::INT16:
    case NumberType::UINT16:
        return 2;

    case NumberType::INT8:
    case NumberType::UINT8:
        return 1;

    default:
        return 0;
    }
}

void decodeBinaryDataArrayHeader(const char *str, VTPBlockHeader *header, char **dataPtr)
{
    size_t bufferSize = 13;
    unsigned char buffer1[bufferSize];

    *dataPtr = (char *) str;

    base64decode(*dataPtr, 16, buffer1, &bufferSize);

    header->blocks                = ((int*) buffer1)[0];
    header->blocksize             = ((int*) buffer1)[1];
    header->last_blocksize        = ((int*) buffer1)[2];
    header->compressed_blocksizes = new int[header->blocks];

    *dataPtr = *dataPtr + 16;
    bufferSize = header->blocks * sizeof(int) + 1;
    unsigned char buffer2[bufferSize];
    base64decode(*dataPtr, 4 * ceil(header->blocks * sizeof(int) / 3.0), buffer2, &bufferSize);

    for (int i=0; i<header->blocks; i++)
        header->compressed_blocksizes[i] = ((int *) buffer2)[i];

    *dataPtr = *dataPtr +  ((int) (4 * ceil(header->blocks * sizeof(int) / 3.0)));
}

int stringsEqualIgnoreCase(const char* s1, const char* s2)
{
    if (s1 == NULL || s2 == NULL)
        return 0;
    char *p1= (char *) s1;
    char *p2= (char *) s2;
    while (p1[0] != 0 && p2[0] != 0)
    {
        if (toupper(p1[0]) != toupper(p2[0]))
            return 0;
        p1++; p2++;
    }
    return (p1[0] == p2[0]);
}

int recognizeType(const char *type)
{
    if(stringsEqualIgnoreCase(type, "float64"))
        return NumberType::FLOAT64;
    else if(stringsEqualIgnoreCase(type, "float32"))
        return NumberType::FLOAT32;
    else if(stringsEqualIgnoreCase(type, "int8"))
        return NumberType::INT8;
    else if(stringsEqualIgnoreCase(type, "uint8"))
        return NumberType::UINT8;
    else if(stringsEqualIgnoreCase(type, "int16"))
        return NumberType::INT16;
    else if(stringsEqualIgnoreCase(type, "uint16"))
        return NumberType::UINT16;
    else if(stringsEqualIgnoreCase(type, "int32"))
        return NumberType::INT32;
    else if(stringsEqualIgnoreCase(type, "uint32"))
        return NumberType::UINT32;
    else
        printf("[Error] Unrecognized number type '%s'\n", type);
    return NumberType::UNDEFINED;
}

void readDataArray(pugi::xml_node *node, VTPDataArray *dataArray)
{
	dataArray->type               = recognizeType(node->attribute("type").as_string());
	dataArray->numberOfComponents = node->attribute("NumberOfComponents").as_int();

    const char *formatStr = node->attribute("format").as_string();

	int format;
    if (formatStr[0] == 'b' || formatStr[0] == 'B')
        format = ArrayFormat::BINARY;
    else if (formatStr[0] == 'a' || formatStr[0] == 'A')
        format = ArrayFormat::ASCII;
    else
    {
        printf("[Error] Cannot recognize format '%s'\n", formatStr);
        return;
    }

    char *data = (char *) node->text().as_string();

	if (format == ArrayFormat::BINARY)
    {
        char *pointData;
        VTPBlockHeader header;
        decodeBinaryDataArrayHeader(data, &header, &pointData);

        int totalDataSize=0;
        size_t totalCompressedSize=0;
        for (int i=0; i<header.blocks; i++)
        {
            if (i==header.blocks-1 && i>0)
                totalDataSize += header.last_blocksize;
            else
                totalDataSize += header.blocksize;
            totalCompressedSize += header.compressed_blocksizes[i];
        }
		unsigned char *decoded = (unsigned char *) malloc(totalCompressedSize);

        base64decode(pointData, 4*ceil(totalCompressedSize/3), decoded, &totalCompressedSize);

        dataArray->data = malloc(totalDataSize);

        int blockOffset = 0, decodedOffset = 0;
        for (int i=0; i<header.blocks; i++)
        {
            uLongf blockSize;
            if (i==header.blocks-1 && i>0)
                blockSize = (uLongf) header.last_blocksize;
            else
                blockSize = (uLongf) header.blocksize;
            size_t compressedBlockSize = header.compressed_blocksizes[i];
            uncompress((unsigned char *) dataArray->data+blockOffset, &blockSize, decoded+decodedOffset, compressedBlockSize);
            blockOffset   += blockSize;
            decodedOffset += header.compressed_blocksizes[i];
        }
		free(decoded);
        delete[] header.compressed_blocksizes;

    }
    else //ASCII
    {
		char * inputData = data;
		if (typeSize(dataArray->type) < 8 && dataArray->type != NumberType::FLOAT32)
        {
			int *parsedData = (int*) malloc(sizeof(int) * dataArray->size * dataArray->numberOfComponents);
            for (int i=0; i<dataArray->size; i++)
			{
                for (int c = 0; c<dataArray->numberOfComponents; c++)
                {
                    char *t;
                    parsedData[i*dataArray->numberOfComponents+c] = strtol(inputData,&t, 10);
                    if (t==inputData)
                    {
                        printf("[Error] Failed to read integer ASCII input (Array='%s', Element=%d)\n", dataArray->arrayName, i);
                        break;
                    }
                    inputData = t;
                }
			}
            dataArray->data = parsedData;
            dataArray->type = NumberType::INT32;
		}
		else if (dataArray->type == NumberType::FLOAT32 || dataArray->type == NumberType::FLOAT64)
		{
			double *parsedData = (double*) malloc(sizeof(double) * dataArray->size * dataArray->numberOfComponents);
			for (int i=0; i<dataArray->size; i++)
			{
                for (int c = 0; c<dataArray->numberOfComponents; c++)
                {
                    char *t;
                    parsedData[i*dataArray->numberOfComponents+c] = strtod(inputData,&t);
                    if (t==inputData)
                    {
                        printf("[Error] Failed to read double ASCII input (Array='%s', Element=%d)\n%s\n", dataArray->arrayName, i, inputData);
                        break;
                    }
                    inputData = t;
                }
			}
            dataArray->data =parsedData;
            dataArray->type = NumberType::FLOAT64;
		}
    }
}

void readPropertyArray(pugi::xml_node *node, VTPDataArray *propertyDataArray)
{
  	string nameStr = string((char *) node->attribute("Name").as_string());
		propertyDataArray->arrayName = new char[nameStr.size()+1];
		strcpy(propertyDataArray->arrayName, nameStr.c_str());

    readDataArray(node, propertyDataArray);
}

void readPositionArray(pugi::xml_document *doc, VTPDataArray *positionDataArray)
{
	pugi::xml_node piece = doc->select_single_node("/VTKFile/PolyData/Piece").node();

  int numberOfPoints = piece.attribute("NumberOfPoints").as_int();
  pugi::xml_node positionArrayNode = piece.select_single_node("Points/DataArray").node();

	positionDataArray->arrayName = (char *) "position";
  positionDataArray->size = numberOfPoints;

    readDataArray(&positionArrayNode, positionDataArray);
}

vector<VTPDataArray> readParticleData(const char* filename)
{
  vector<VTPDataArray> particleData = vector<VTPDataArray>();
	pugi::xml_document doc;
	doc.load_file(filename);

	VTPDataArray pos;
	readPositionArray(&doc, &pos);
  particleData.push_back(pos);

	pugi::xpath_node_set pointDataArrays = doc.select_nodes("/VTKFile/PolyData/Piece/PointData/DataArray");

  for (unsigned int i=0; i<pointDataArrays.size(); i++)
  {
    pugi::xml_node node = pointDataArrays[i].node();
    VTPDataArray property;
		property.size = pos.size;
		readPropertyArray(&node, &property);
    particleData.push_back(property);
	}

  return particleData;
}

ParticlesDataMutable* readVTP(const char* filename, const bool headersOnly,std::ostream* errorStream){

    ParticlesDataMutable* simple = headersOnly ? new ParticleHeaders: create();

    vector<VTPDataArray> pData = readParticleData(filename);
    simple->addParticles(pData[0].size);

    for(int attrIndex = 0; attrIndex < pData.size(); attrIndex++){
      ParticleAttribute attr;
      int numComponents = pData[attrIndex].numberOfComponents;
			if(attrIndex==0)
			{
				attr = simple->addAttribute(pData[attrIndex].arrayName, VECTOR, numComponents);
			}
      else if(pData[attrIndex].type==NumberType::FLOAT32 || pData[attrIndex].type==NumberType::FLOAT64)
			{
        attr = simple->addAttribute(pData[attrIndex].arrayName, FLOAT, numComponents);
			}
      //else
        //attr = simple->addAttribute(pData[attrIndex].arrayName, INT, numComponents);

			if(pData[attrIndex].type==NumberType::FLOAT32)
      {
        float *data = (float *) pData[attrIndex].data;
        for(int partIndex = 0; partIndex < simple->numParticles(); partIndex++){
          for(int dim = 0; dim < attr.count; dim++){
            simple->dataWrite<float>(attr, partIndex)[dim] = data[partIndex*numComponents+dim];
          }
        }
      } else if (pData[attrIndex].type==NumberType::FLOAT64)
      {
        double *data = (double *) pData[attrIndex].data;
        for(int partIndex = 0; partIndex < simple->numParticles(); partIndex++){
          for(int dim = 0; dim < attr.count; dim++){
            simple->dataWrite<float>(attr, partIndex)[dim] = (float)data[partIndex*numComponents+dim];
          }
        }
      }
      //free(pData[attrIndex].data);
    }

		// Add id attribute
		ParticleAttribute attr;
		attr = simple->addAttribute("id", INT, 1);
		for(int partIndex = 0; partIndex < simple->numParticles(); partIndex++){
			simple->dataWrite<int>(attr, partIndex)[0] = partIndex;
		}

    return simple;
}

/*bool writePDC(const char* filename,const ParticlesData& p,const bool compressed,std::ostream* errorStream){
    auto_ptr<ostream> output(
        compressed ?
        Gzip_Out(filename,ios::out|ios::binary)
        :new std::ofstream(filename,ios::out|ios::binary));

    if(!*output){
        if(errorStream) *errorStream << "Partio Unable to open file " << filename << endl;
        return false;
    }

    // write .pdc header
    write<LITEND>(*output, PDC_MAGIC);
    write<BIGEND>(*output, (int)1); // version
    write<BIGEND>(*output, (int)1); // bitorder
    write<BIGEND>(*output, (int)0); // tmp1
    write<BIGEND>(*output, (int)0); // tmp2
    write<BIGEND>(*output, (int)p.numParticles());
    write<BIGEND>(*output, (int)p.numAttributes());

    for(int attrIndex = 0; attrIndex < p.numAttributes(); attrIndex++){
        ParticleAttribute attr;
        p.attributeInfo(attrIndex,attr);

        // write attribute name
        write<BIGEND>(*output, (int)attr.name.length());
        output->write(attr.name.c_str(), (int)attr.name.length());

        // write type
        int count = 1; // FLOAT
        if(attr.type == VECTOR){
            count = 3;
        }
        write<BIGEND>(*output, (int)(count+2));

        // write data
        for(int partIndex = 0; partIndex < p.numParticles(); partIndex++){
            const float* data = p.data<float>(attr, partIndex);
            for(int dim = 0; dim < count; dim++){
                write<BIGEND>(*output, (double)data[dim]);
            }
        }
    }
    return true;
}*/


}// end of namespace Partio
