#include "xinjingcheng_io.hpp"


void IO_XinJinCheng::controlIO()
{
    count++;
    if (!(count % 1000)) {
        for (auto& io : rx_) {
            printIO(io.get());
        }
        for (auto& io : tx_) {
            printIO(io.get());
        }

        set_zero(bSetZero_CH1, 1);
        set_zero(bSetZero_CH2, 2);
        set_zero(bSetZero_CH3, 3);
        set_zero(bSetZero_CH4, 4);
        set_zero(bSetZero_CH5, 5);
        set_zero(bSetZero_CH6, 6);
        set_zero(bSetZero_CH7, 7);
        set_zero(bSetZero_CH8, 8);
        NECRO_RT(printf("#################################### \n"));
    }
    count %= 100000;
}

void IO_XinJinCheng::bind_pdo()
{
    for (auto& io : tx_) {
        if (io->io_idx == 0x6000 && io->io_subIdx == 1) {
            fAI_CH1_GROSS.data_ = io.get();
        }
        else if (io->io_idx == 0x6001 && io->io_subIdx == 1) {
            fAI_CH2_GROSS.data_ = io.get();
        }
        else if (io->io_idx == 0x6002 && io->io_subIdx == 1) {
            fAI_CH3_GROSS.data_ = io.get();
        }
        else if (io->io_idx == 0x6003 && io->io_subIdx == 1) {
            fAI_CH4_GROSS.data_ = io.get();
        }
        else if (io->io_idx == 0x6004 && io->io_subIdx == 1) {
            fAI_CH5_GROSS.data_ = io.get();
        }
        else if (io->io_idx == 0x6005 && io->io_subIdx == 1) {
            fAI_CH6_GROSS.data_ = io.get();
        }
        else if (io->io_idx == 0x6006 && io->io_subIdx == 1) {
            fAI_CH7_GROSS.data_ = io.get();
        }
        else if (io->io_idx == 0x6007 && io->io_subIdx == 1) {
            fAI_CH8_GROSS.data_ = io.get();
        }
        else if (io->io_idx == 0x6020 && io->io_subIdx == 1) {
            bSetZeroFailed_CH1.data_ = io.get();
        }
        else if (io->io_idx == 0x6020 && io->io_subIdx == 2) {
            bSetZeroFailed_CH2.data_ = io.get();
        }
        else if (io->io_idx == 0x6020 && io->io_subIdx == 3) {
            bSetZeroFailed_CH3.data_ = io.get();
        }
        else if (io->io_idx == 0x6020 && io->io_subIdx == 4) {
            bSetZeroFailed_CH4.data_ = io.get();
        }
        else if (io->io_idx == 0x6020 && io->io_subIdx == 5) {
            bSetZeroFailed_CH5.data_ = io.get();
        }
        else if (io->io_idx == 0x6020 && io->io_subIdx == 6) {
            bSetZeroFailed_CH6.data_ = io.get();
        }
        else if (io->io_idx == 0x6020 && io->io_subIdx == 7) {
            bSetZeroFailed_CH7.data_ = io.get();
        }
        else if (io->io_idx == 0x6020 && io->io_subIdx == 8) {
            bSetZeroFailed_CH8.data_ = io.get();
        }
        else if (io->io_idx == 0x6021 && io->io_subIdx == 1) {
            bSave_Para_Failed_CH1.data_ = io.get();
        }
        else if (io->io_idx == 0x6021 && io->io_subIdx == 2) {
            bSave_Para_Failed_CH2.data_ = io.get();
        }
        else if (io->io_idx == 0x6021 && io->io_subIdx == 3) {
            bSave_Para_Failed_CH3.data_ = io.get();
        }
        else if (io->io_idx == 0x6021 && io->io_subIdx == 4) {
            bSave_Para_Failed_CH4.data_ = io.get();
        }
        else if (io->io_idx == 0x6021 && io->io_subIdx == 5) {
            bSave_Para_Failed_CH5.data_ = io.get();
        }
        else if (io->io_idx == 0x6021 && io->io_subIdx == 6) {
            bSave_Para_Failed_CH6.data_ = io.get();
        }
        else if (io->io_idx == 0x6021 && io->io_subIdx == 7) {
            bSave_Para_Failed_CH7.data_ = io.get();
        }
        else if (io->io_idx == 0x6021 && io->io_subIdx == 8) {
            bSave_Para_Failed_CH8.data_ = io.get();
        }
        else if (io->io_idx == 0x6022 && io->io_subIdx == 0) {
            fParameterToBeRead.data_ = io.get();
        }
    }
    for (auto& io : rx_) {
        if (io->io_idx == 0x7000 && io->io_subIdx == 1) {
            bSetZero_CH1.data_ = io.get();
        }
        else if (io->io_idx == 0x7000 && io->io_subIdx == 2) {
            bSetZero_CH2.data_ = io.get();
        }
        else if (io->io_idx == 0x7000 && io->io_subIdx == 3) {
            bSetZero_CH3.data_ = io.get();
        }
        else if (io->io_idx == 0x7000 && io->io_subIdx == 4) {
            bSetZero_CH4.data_ = io.get();
        }
        else if (io->io_idx == 0x7000 && io->io_subIdx == 5) {
            bSetZero_CH5.data_ = io.get();
        }
        else if (io->io_idx == 0x7000 && io->io_subIdx == 6) {
            bSetZero_CH6.data_ = io.get();
        }
        else if (io->io_idx == 0x7000 && io->io_subIdx == 7) {
            bSetZero_CH7.data_ = io.get();
        }
        else if (io->io_idx == 0x7000 && io->io_subIdx == 8) {
            bSetZero_CH8.data_ = io.get();
        }
        else if (io->io_idx == 0x7000 && io->io_subIdx == 9) {
            usSetZero_ALL.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 1) {
            bSetZero_PV_CH1.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 2) {
            bSetZero_PV_CH2.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 3) {
            bSetZero_PV_CH3.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 4) {
            bSetZero_PV_CH4.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 5) {
            bSetZero_PV_CH5.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 6) {
            bSetZero_PV_CH6.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 7) {
            bSetZero_PV_CH7.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 8) {
            bSetZero_PV_CH8.data_ = io.get();
        }
        else if (io->io_idx == 0x7002 && io->io_subIdx == 9) {
            usSetZero_PV_ALL.data_ = io.get();
        }
        else if (io->io_idx == 0x7003 && io->io_subIdx == 0) {
            udParameterToBeSet.data_ = io.get();
        }
        else if (io->io_idx == 0x7004 && io->io_subIdx == 0) {
            fParameterSetValue.data_ = io.get();
        }
    }
}


void IO_XinJinCheng::senCalAll()
{
    switch (channelType)
    {
    case 1:
        if (sensitivity_calibration(
            double_addr(0x93, channelType), 1,
            double_addr(0x95, channelType), 255,
            double_addr(0x97, channelType), 100,
            double_addr(0x94, channelType), 1.92786))
        {
            calibrationState = 0;
            channelType = 2;
        }
        break;
    case 2:
        if (sensitivity_calibration(
            double_addr(0x93, channelType), 1,
            double_addr(0x95, channelType), 255,
            double_addr(0x97, channelType), 100,
            double_addr(0x94, channelType), 1.92786))
        {
            calibrationState = 0;
            channelType = 3;
        }
        break;
    case 3:
        if (sensitivity_calibration(
            double_addr(0x93, channelType), 1,
            double_addr(0x95, channelType), 255,
            double_addr(0x97, channelType), 100,
            double_addr(0x94, channelType), 1.92786))
        {
            calibrationState = 0;
            channelType = 4;
        }
        break;
    case 4:
        if (sensitivity_calibration(
            double_addr(0x93, channelType), 1,
            double_addr(0x95, channelType), 255,
            double_addr(0x97, channelType), 100,
            double_addr(0x94, channelType), 1.92786))
        {
            calibrationState = 0;
            channelType = 5;
        }
        break;
    case 5:
        if (sensitivity_calibration(
            double_addr(0x93, channelType), 1,
            double_addr(0x95, channelType), 255,
            double_addr(0x97, channelType), 100,
            double_addr(0x94, channelType), 1.92786))
        {
            calibrationState = 0;
            channelType = 6;
        }
        break;
    case 6:
        if (sensitivity_calibration(
            double_addr(0x93, channelType), 1,
            double_addr(0x95, channelType), 255,
            double_addr(0x97, channelType), 100,
            double_addr(0x94, channelType), 1.92786))
        {
            calibrationState = 0;
            channelType = 9;
        }
        break;
        // case 7:
        //     if (sensitivity_calibration(
        //         double_addr(0x93, channelType), 1,
        //         double_addr(0x95, channelType), 255,
        //         double_addr(0x97, channelType), 100,
        //         double_addr(0x94, channelType), 1.92786))
        //     {
        //         calibrationState = 0;
        //         channelType = 8;
        //     }
        //     break;
        // case 8:
        //     if (sensitivity_calibration(
        //         double_addr(0x93, channelType), 1,
        //         double_addr(0x95, channelType), 255,
        //         double_addr(0x97, channelType), 100,
        //         double_addr(0x94, channelType), 1.92786))
        //     {
        //         calibrationState = 0;
        //         channelType = 9;
        //     }
        //     break;
    case 9:
        break;
    default:
        break;
    }
}


uint32_t IO_XinJinCheng::double_addr(uint32_t type, int channel)
{
    return (type + (channel - 1) * 0x00ff) * 2;
}


bool IO_XinJinCheng::sensitivity_calibration
(uint32_t methAddr, float methValue,
    uint32_t zeroAddr, float zeroValue,
    uint32_t calpAddr, float calpValue,
    uint32_t mv_vAddr, float mv_vValue)
{
    bool sensitivitySuccess = false;
    switch (calibrationState)
    {
    case 0:
        if (write_param(methAddr, methValue)) {
            calibrationState = 1;
            writeState = 0;
        }
        break;
    case 1:
        if (write_param(zeroAddr, zeroValue)) {
            calibrationState = 2;
            writeState = 0;
        }
        break;
    case 2:
        if (write_param(calpAddr, calpValue)) {
            calibrationState = 3;
            writeState = 0;
        }
        break;
    case 3:
        if (write_param(mv_vAddr, mv_vValue)) {
            calibrationState = 4;
            writeState = 0;
        }
        break;
    case 4:
        sensitivitySuccess = true;
        break;
    default:
        break;
    }
    return sensitivitySuccess;
}


bool IO_XinJinCheng::write_param(uint32_t toBeSet, float setValue)
{
    bool calibrationSucess = false;
    switch (writeState)
    {
    case 0:
        udParameterToBeSet.set((uint32_t)65535);
        writeState = 1;
        break;
    case 1:
        fParameterSetValue.set(setValue);
        writeState = 2;
        break;
    case 2:
        udParameterToBeSet.set(toBeSet);
        writeState = 3;
        break;
    case 3:
        calibrationSucess = true;
        break;
    default:
        break;
    }
    return calibrationSucess;
}


bool IO_XinJinCheng::set_zero(DataIO& ioBit, int channel)
{
    int ch = channel - 1;
    bool setZeroSucess = false;
    switch (setZeroState[ch])
    {
    case 0:
        ioBit.set((bool)1);
        setZeroState[ch] = 1;
        break;
    case 1:
        ioBit.set((bool)0);
        setZeroState[ch] = 2;
        break;
    case 2:
        setZeroSucess = true;
        break;
    default:
        break;
    }
    return setZeroSucess;
}

