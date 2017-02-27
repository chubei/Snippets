#include "stdafx.h"
#include "ParameterPack.hpp"
#include "ParameterPackVisualizer.hpp"
#include <opencv2\highgui.hpp>

ParameterPackVisualizer::ParameterPackVisualizer(ParameterPack & pack, CallbackType const & callback) : _pPack(&pack), _callback(callback) {
    if (_pPack)
        _trackBarData.resize(_pPack->_parameters.size());
}

void ParameterPackVisualizer::setMaxValue(void * pData, double maxValue) {
    if (!_pPack)
        return;

    for (size_t i = 0; i < _pPack->_parameters.size(); ++i)
        if (_pPack->_parameters[i]._pData == pData) {
            _trackBarData[i].maxValue = maxValue;
            return;
        }
}

void ParameterPackVisualizer::setPrecision(void * pData, double precision) {
    if (!_pPack)
        return;

    for (size_t i = 0; i < _pPack->_parameters.size(); ++i)
        if (_pPack->_parameters[i]._pData == pData) {
            _trackBarData[i].precision = precision;
            return;
        }
}

void ParameterPackVisualizer::showInNewWindow(const char * name) {
    cv::namedWindow(name);
    showInExistingWindow(name);
}

void ParameterPackVisualizer::showInExistingWindow(const char * name) {
    if (!_pPack)
        return;

    for (size_t i = 0; i < _pPack->_parameters.size(); ++i) {
        auto& parameter = _pPack->_parameters[i];
        auto& bar_data = _trackBarData[i];

        switch (parameter._type) {
        case ParameterPack::ParameterType::Bool:
            bar_data.value = parameter.get<bool>() ? 1 : 0;
            cv::createTrackbar(parameter._name, name, &bar_data.value, 2, &ParameterPackVisualizerCallback, this);
            break;
        case ParameterPack::ParameterType::Int32:
            bar_data.value = parameter.get<int32_t>();
            cv::createTrackbar(parameter._name, name, &bar_data.value, int(bar_data.maxValue) + 1, &ParameterPackVisualizerCallback, this);
            break;
        case ParameterPack::ParameterType::Double:
            bar_data.value = int(parameter.get<double>() / bar_data.precision);
            {
                cv::String bar_name = parameter._name;
                char precision_str[30];
                sprintf_s(precision_str, "*%.2f", bar_data.precision);
                bar_name += cv::String(precision_str);
                int bar_count = int(bar_data.maxValue / bar_data.precision) + 1;
                cv::createTrackbar(bar_name, name, &bar_data.value, bar_count, &ParameterPackVisualizerCallback, this);
            }
            break;
        default:
            break;
        }
    }
}

void ParameterPackVisualizer::onTrackBarChange() {
    if (!_pPack)
        return;

    for (size_t i = 0; i < _pPack->_parameters.size(); ++i) {
        auto& parameter = _pPack->_parameters[i];
        auto& bar_data = _trackBarData[i];

        switch (parameter._type) {
        case ParameterPack::ParameterType::Bool:
            parameter.set<bool>(bar_data.value != 0);
            break;
        case ParameterPack::ParameterType::Int32:
            parameter.set<int32_t>(bar_data.value);
            break;
        case ParameterPack::ParameterType::Double:
            parameter.set<double>(bar_data.value * bar_data.precision);
            break;
        default:
            break;
        }
    }

    _callback(*_pPack);
}

void ParameterPackVisualizerCallback(int, void * user_data) {
    ((ParameterPackVisualizer*)user_data)->onTrackBarChange();
}
