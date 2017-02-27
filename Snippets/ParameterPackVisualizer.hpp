#pragma once
#include <functional>

class ParameterPack;

class ParameterPackVisualizer {
public:
    using CallbackType = std::function<void(ParameterPack&)>;

public:
    explicit ParameterPackVisualizer(ParameterPack& pack, CallbackType const& callback = [](ParameterPack&) {});
    ParameterPackVisualizer(ParameterPackVisualizer const&) = delete;
    ParameterPackVisualizer& operator=(ParameterPackVisualizer const&) = delete;

    void setCallback(CallbackType const& callback) { _callback = callback; }
    void setMaxValue(void* pData, double maxValue);
    void setPrecision(void* pData, double precision);

    void showInNewWindow(const char* name);
    void showInExistingWindow(const char* name);

private:
    ParameterPack* _pPack{ nullptr };
    CallbackType _callback{ [](ParameterPack&) {} };

    struct TrackBarData {
        int value{ 0 };
        double maxValue{ 100 };
        double precision{ 1 };
    };
    std::vector<TrackBarData> _trackBarData;

private:
    void onTrackBarChange();
    friend void ParameterPackVisualizerCallback(int, void*);
};

void ParameterPackVisualizerCallback(int, void* user_data);
