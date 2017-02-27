#pragma once

#include <stdint.h>
#include <vector>

class ParameterPack {
public:
    enum class ParameterType { Bool, Int32, Double };

    struct Parameter {
    public:
        Parameter(ParameterType type, const char* name, void* pData);

    public:
        ParameterType _type;
        const char* _name;
        void* _pData;

        template<class _T>
        void set(_T const& value, bool* const succeed = nullptr);

        template<class _T>
        _T get(bool* const succeed = nullptr) const;
    };

public:
    ParameterPack() {}
    virtual ~ParameterPack() {}
    ParameterPack(ParameterPack const&) = default;
    ParameterPack(ParameterPack&&) = default;
    ParameterPack& operator=(ParameterPack const&) = default;
    ParameterPack& operator=(ParameterPack&&) = default;

public:
    std::vector<Parameter> _parameters;

protected:
    template<class _Type>
    void registerParameter(_Type& para, const char* name, bool* const succeed = nullptr);
};

namespace Detail {
    template<class _Type>
    inline void set(_Type const& value, ParameterPack::ParameterType type, void* pData, bool* const succeed) {
        if (succeed)
            *succeed = false;
    }

    template<class _Type>
    inline _Type get(ParameterPack::ParameterType type, void const* pData, bool* const succeed) {
        if (succeed)
            *succeed = false;
        return _Type();
    }

    template<class _T>
    inline void registerParameter(std::vector<ParameterPack::Parameter>& parameters, const char* name, _T& para, bool* const succeed) {
        if (succeed)
            *succeed = false;
    }

#define SetImplement(Type, Flag)\
    template<>\
    inline void set<Type>(const Type& value, ParameterPack::ParameterType type, void* pData, bool* const succeed) {\
        if (type != Flag || !pData) {\
            if (succeed)\
                *succeed = false;\
            return;\
        }\
\
        *(Type*)(pData) = value;\
        if (succeed)\
            *succeed = true;\
    }

#define GetImplement(Type, Flag)\
    template<>\
    inline Type get<Type>(ParameterPack::ParameterType type, void const* pData, bool* const succeed) {\
        if (type != Flag || !pData) {\
            if (succeed)\
                *succeed = false;\
            return Type();\
        }\
\
        if (succeed)\
            *succeed = true;\
        return *(Type const*)(pData);\
    }

#define RegImplement(Type, Flag)\
    template<>\
    inline void registerParameter<Type>(std::vector<ParameterPack::Parameter>& parameters, const char* name, Type& para, bool* const succeed) {\
        if (succeed)\
            *succeed = true;\
        parameters.emplace_back(Flag, name, &para);\
    }

#define ParameterImpl(Type, Flag)\
    SetImplement(Type, Flag);\
    GetImplement(Type, Flag);\
    RegImplement(Type, Flag)

    ParameterImpl(bool, ParameterPack::ParameterType::Bool);
    ParameterImpl(int32_t, ParameterPack::ParameterType::Int32);
    ParameterImpl(double, ParameterPack::ParameterType::Double);

#undef ParameterImpl
#undef SetImplement
#undef GetImplement
#undef RegImplement
}

template<class _T>
inline void ParameterPack::Parameter::set(_T const& value, bool* const succeed) {
    Detail::set<_T>(value, this->_type, this->_pData, succeed);
}

template<class _T>
inline _T ParameterPack::Parameter::get(bool* const succeed) const {
    return Detail::get<_T>(this->_type, this->_pData, succeed);
}

template<class _T>
inline void ParameterPack::registerParameter(_T& para, const char* name, bool* const succeed) {
    Detail::registerParameter<_T>(_parameters, name, para, succeed);
}
