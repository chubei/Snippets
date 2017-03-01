#include "ParameterPack.hpp"

ParameterPack::Parameter::Parameter(ParameterType type, const char* name, void* pData) :_type(type), _name(name), _pData(pData) {
}
