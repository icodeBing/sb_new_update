#pragma once

#include "io_operation.hpp"


#define CHANNEL_SIZE 8

class IO_XinJinCheng : public IoController
{
public:
    // rx pdo
    DataIO bSetZero_CH1;
    DataIO bSetZero_CH2;
    DataIO bSetZero_CH3;
    DataIO bSetZero_CH4;
    DataIO bSetZero_CH5;
    DataIO bSetZero_CH6;
    DataIO bSetZero_CH7;
    DataIO bSetZero_CH8;
    DataIO usSetZero_ALL;
    DataIO bSetZero_PV_CH1;
    DataIO bSetZero_PV_CH2;
    DataIO bSetZero_PV_CH3;
    DataIO bSetZero_PV_CH4;
    DataIO bSetZero_PV_CH5;
    DataIO bSetZero_PV_CH6;
    DataIO bSetZero_PV_CH7;
    DataIO bSetZero_PV_CH8;
    DataIO usSetZero_PV_ALL;
    DataIO udParameterToBeSet;
    DataIO fParameterSetValue;

    // tx pdo
    DataIO fAI_CH1_GROSS;
    DataIO fAI_CH2_GROSS;
    DataIO fAI_CH3_GROSS;
    DataIO fAI_CH4_GROSS;
    DataIO fAI_CH5_GROSS;
    DataIO fAI_CH6_GROSS;
    DataIO fAI_CH7_GROSS;
    DataIO fAI_CH8_GROSS;
    DataIO bSetZeroFailed_CH1;
    DataIO bSetZeroFailed_CH2;
    DataIO bSetZeroFailed_CH3;
    DataIO bSetZeroFailed_CH4;
    DataIO bSetZeroFailed_CH5;
    DataIO bSetZeroFailed_CH6;
    DataIO bSetZeroFailed_CH7;
    DataIO bSetZeroFailed_CH8;
    DataIO bSave_Para_Failed_CH1;
    DataIO bSave_Para_Failed_CH2;
    DataIO bSave_Para_Failed_CH3;
    DataIO bSave_Para_Failed_CH4;
    DataIO bSave_Para_Failed_CH5;
    DataIO bSave_Para_Failed_CH6;
    DataIO bSave_Para_Failed_CH7;
    DataIO bSave_Para_Failed_CH8;
    DataIO fParameterToBeRead;
public:
    void controlIO() override;
    void bind_pdo() override;

    void senCalAll();

    uint32_t double_addr(uint32_t type, int channel);
    bool sensitivity_calibration(
        uint32_t methAddr, float methValue,
        uint32_t zeroAddr, float zeroValue,
        uint32_t calpAddr, float calpValue,
        uint32_t mv_vAddr, float mv_vValue);
    bool write_param(uint32_t toBeSet, float setValue);
    bool set_zero(DataIO& ioBit, int channel);
private:
    int count = 0;
    int writeState = 0;         // 0-3
    int calibrationState = 0;   // 0-4
    int channelType = 1;        // 1-9
    int setZeroState[CHANNEL_SIZE] = { 0 };
};
