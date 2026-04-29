#include "xinjingcheng_io.hpp"


// 新劲成设备的周期 IO 处理入口。
// 这个函数当前并不做闭环控制，主要做两件事：
// 1. 打印已经映射完成的 PDO 值，方便调试；
// 2. 通过 rx PDO 给每个通道发送一次性的清零命令。
void IO_XinJinCheng::controlIO()
{
    count++;
    if (!(count % 1000)) {
        // rx_：主站写给从站的输出 PDO
        for (auto& io : rx_) {
            printIO(io.get());
        }
        // tx_：从站返回给主站的输入 PDO
        for (auto& io : tx_) {
            printIO(io.get());
        }

        // 依次给 8 个通道发送“清零脉冲”。
        // set_zero() 内部是一个很小的状态机：先写 1，再写 0，随后保持完成状态。
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

// 把扫描到的原始 PDO 项绑定到具名的 DataIO 成员上。
// 完成绑定后，上层逻辑就可以直接按语义读写，
// 不需要每个周期都去匹配 (index, subindex)。
void IO_XinJinCheng::bind_pdo()
{
    // tx PDO：从站返回给主站的数据
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

    // rx PDO：主站下发给从站的命令
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


// 按通道依次执行灵敏度标定。
// channelType 表示当前处理到哪个通道。
// sensitivity_calibration() 本身也是状态机，
// 所以只有当前通道整套流程结束后，才会切到下一个通道。
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


// 生成某个通道对应的参数地址。
// 从这里的计算方式看，设备大概率按每通道 0x00ff 的间隔分配参数区，
// 最后的 * 2 是把字单位偏移转换成协议里使用的地址形式。
uint32_t IO_XinJinCheng::double_addr(uint32_t type, int channel)
{
    return (type + (channel - 1) * 0x00ff) * 2;
}


// 单个通道灵敏度标定的多步写参数流程。
// 状态流转如下：
// 0 -> 写方法参数
// 1 -> 写零点参数
// 2 -> 写标定百分比
// 3 -> 写 mV/V 参数
// 4 -> 整个流程完成
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
        // 第一步：写入标定方法参数。
        if (write_param(methAddr, methValue)) {
            calibrationState = 1;
            writeState = 0;
        }
        break;
    case 1:
        // 第二步：写入零点参数。
        if (write_param(zeroAddr, zeroValue)) {
            calibrationState = 2;
            writeState = 0;
        }
        break;
    case 2:
        // 第三步：写入标定百分比参数。
        if (write_param(calpAddr, calpValue)) {
            calibrationState = 3;
            writeState = 0;
        }
        break;
    case 3:
        // 第四步：写入 mV/V 参数。
        if (write_param(mv_vAddr, mv_vValue)) {
            calibrationState = 4;
            writeState = 0;
        }
        break;
    case 4:
        // 当前通道整套标定流程完成。
        sensitivitySuccess = true;
        break;
    default:
        break;
    }
    return sensitivitySuccess;
}


// 设备特定的参数写入握手流程。
// 这里不是一个周期内同时下发“参数地址 + 参数值”，
// 而是拆成固定的 4 个阶段分多周期完成：
// 0 -> 先把参数选择器清成 65535
// 1 -> 再写参数值
// 2 -> 最后写目标参数地址
// 3 -> 返回写入完成
bool IO_XinJinCheng::write_param(uint32_t toBeSet, float setValue)
{
    bool calibrationSucess = false;
    switch (writeState)
    {
    case 0:
        // 先把待写参数选择器复位。
        udParameterToBeSet.set((uint32_t)65535);
        writeState = 1;
        break;
    case 1:
        // 再把参数值写入数值寄存器。
        fParameterSetValue.set(setValue);
        writeState = 2;
        break;
    case 2:
        // 最后写入真正的参数地址，触发从站更新该参数。
        udParameterToBeSet.set(toBeSet);
        writeState = 3;
        break;
    case 3:
        // 本轮握手完成。
        calibrationSucess = true;
        break;
    default:
        break;
    }
    return calibrationSucess;
}


// 单个通道的一次性清零命令发生器。
// 它会把对应 bit 按 0 -> 1 -> 0 的方式翻转一次后停止，
// 这通常说明从站期望的是“边沿触发命令”，而不是一直保持高电平。
bool IO_XinJinCheng::set_zero(DataIO& ioBit, int channel)
{
    int ch = channel - 1;
    bool setZeroSucess = false;
    switch (setZeroState[ch])
    {
    case 0:
        // 先把清零命令位置 1。
        ioBit.set((bool)1);
        setZeroState[ch] = 1;
        break;
    case 1:
        // 下一周期再拉低，形成一个完整脉冲。
        ioBit.set((bool)0);
        setZeroState[ch] = 2;
        break;
    case 2:
        // 命令已经发送完成。
        setZeroSucess = true;
        break;
    default:
        break;
    }
    return setZeroSucess;
}
