/**
 * Created by Alvin Liu on 2016/4/27.
 * 解析与执行命令
 * 命令解析、命令执行以及数据解析模块
 *
 * 当前的服务器命令有：
 *
 *
 *
 * 当前的下位机串口命令有：
 * 命令格式
 * 头             栈长度     命令字   数据          校验和      结束字节   意义
 * 0x66  0xaa     0x01      0x01     "0"           0x##       0xfc      获取超声波测距的结果，数据：超声波模块索引
 * 0x66  0xaa     0x08      0x03     "+100-100"    0x##       0xfc      设置电机速度，数据：两个电机速度(0~255)，+-表示正反转
 *
 * 当前系统的数据格式有：
 * 头             栈长度     数据字   数据          校验和      结束字节   意义
 * 0x66  0xaa     0x08      0x80     "5fdfasdgag"  0x00       0xfc      图像数据
 * 0x66  0xaa     0x##      0x81     10 个字符串    0x##       0xfc      JY901数据（3个加速度，3个角速度，3个角度[pitch、roll、yaw]，温度）
 * 0x66  0xaa     0x##      0x82     14 个字符串    0x##       0xfc      arduino数据（湿度、温度、8个红外（1表示不正常、0正常），4个超声波）
 * 0x66  0xaa     0x##      0x83     2 个字符串     0x##       0xfc      arduino测速数据（左右轮编码计数器数值）
 *
 * 0x66  0xaa     0x##      0xa0     12 个字符串    0x##       0xfc      融合滤波后的Odometry数据（3个加速度，3个角速度，姿态pitch, roll, yaw, 离起点的位置: x, y, z;）
 * 0x66  0xaa     0x##      0xa1     12 个字符串    0x##       0xfc      轮式里程计的数据
 * 0x66  0xaa     0x##      0xa2     12 个字符串    0x##       0xfc      VO的数据
 */

'use strict'

const fs = require('fs')
const path = require('path')
const motorPort = require('./serial').motorPort
const Rx = require('rxjs/Rx')
const { crc8 } = require('./crc')

// 命令的一些固定字节
const HEAD1 = '\x66'
const HEAD2 = '\xaa'
const END = '\xfc'

// 一些常用函数

/**
 * Byte 转换 为 字符 ('\xaa'等)
 * @param abyte
 * @returns {string|String}
 * @constructor
 */
function ByteToString (abyte) {
  let buf = Buffer.alloc(1)
  buf.writeUInt8(abyte)
  return buf.toString('binary')
}

/**
 * 命令字对应的命令，返回的对象有两个属性：
 * level表示这个命令的级别，级别-1说明命令错误，级别0是对小车的应用命令，级别1则是底层的发送给电机驱动板串口的命令
 * description是这个命令的描述
 * @type {Map}
 */
const CMDMAP = new Map([
  ['\x00', {level: -1, description: '未能解析参数'}],
  ['\x03', {level: 1, description: '设置电机速度'}]
])
exports.CMDMAP = CMDMAP

function CommandFactory (cmdLine, cmdID, data) {
  const cmdInfo = CMDMAP.get(cmdID)
  return new Command(cmdLine, cmdID, cmdInfo.level, cmdInfo.description, data)
}

class Command {
  /**
   * 一个命令的构造函数
   * @param cmdLine 命令行字符串，包括命令头，命令结束字节等
   * @param cmdID 命令字
   * @param level 命令的级别
   * @param description 命令的描述
   * @param data 命令中的数据
   */
  constructor (cmdLine, cmdID, level, description, data) {
    this.cmdLine = cmdLine
    this.cmdID = cmdID
    this.level = level
    this.description = description
    this.data = data
  }
}
exports.Command = Command

// /**
//  * 发送电机控制命令信息，与编码器计数的流进行组合从而判断电机正反转
//  * @type {Subject<object>} {speedL: 50, speedR: -50}
//  */
// const motorCmd$ = new Rx.BehaviorSubject({speedL: 0, speedR: 0})
let speedL = 0
let speedR = 0
/**
 * 执行命令
 * @param cmd Command实例
 */
exports.executeCommand = function (cmd) {
  switch (cmd.level) {
    case 0:  // 对小车的一些高级命令，如命令它自主移动等等
    {

    }
      break
    case 1:  // 发送给电机驱动板串口的命令
    {
      /**
       * 电机驱动板的串口命令格式（使用购买的驱动板，格式已锁定，所以不同于本应用的指令）如下：
       * 0xff ID+CMD CH DATAL DATAH
       * 0xff是命令头 ID是电机驱动板的编号，这里恒为0即可，占该字节前4位，CMD是命令字占后4位，
       * CH是通道号（如直流电机有4个通道，0就是第一个），DATAL DATAH即为数据，按小端方式存放
       *
       * 由于ID恒为0，所以命令字直接设置为1个字节：
       * 0x01 设置舵机速度
       * 0x02 设置舵机位置
       * 0x03 设置直流电机速度
       * 0x04 设置步进电机速度
       * 0x05 设置步进电机步数
       * 0x09 设置当前动作组
       * 0x0b 急停
       */
      let motorLeftCmd = Buffer.alloc(5)
      let motorRightCmd = Buffer.alloc(5)
      // \xff \x03 \x00
      motorLeftCmd.write('\xff', 0, 'binary')
      motorLeftCmd.write(cmd.cmdID, 1, 'binary')
      motorLeftCmd.write('\x00', 2, 'binary')
      // \xff \x03 \x01
      motorRightCmd.write('\xff', 0, 'binary')
      motorRightCmd.write(cmd.cmdID, 1, 'binary')
      motorRightCmd.write('\x01', 2, 'binary')

      const speedLAbs = parseInt(cmd.data.substr(1, 3))
      const speedRAbs = parseInt(cmd.data.substr(5, 3))
      
      speedL = cmd.data[0] === '-' ? -speedLAbs : speedLAbs
      speedR = cmd.data[4] === '-' ? -speedRAbs : speedRAbs

      motorLeftCmd.writeInt16LE(speedL, 3)
      motorRightCmd.writeInt16LE(speedR, 3)

      motorPort.write(motorLeftCmd)
      motorPort.write(motorRightCmd)
    }
      break
  }
}


/**
 * 解析命令字符串
 * 头             栈长度     命令字   数据          校验和      结束字节
 * 0x66  0xaa     0x08      0x01     "+100-100"     0x##      0xfc
 * @param cmdLine {string} binary
 */
exports.parseCommand = function (cmdLine) {
  let cmdID = cmdLine[3]
  let cmdLen = Buffer.from(cmdLine[2], 'binary')[0]
  let data = cmdLine.substr(4, cmdLen)

  let checkSum = cmdLine.slice(-2, -1)
  let checkData = cmdLine.slice(2, -2)  // 栈长度 + 命令字 + 数据

  // let b = Buffer.from(checkData, 'binary')

  if (ByteToString(crc8(checkData)) !== checkSum)  // 校验和不匹配
    cmdID = '\x00'

  return CommandFactory(cmdLine, cmdID, data)

}

/**
 * JY901 串口信息解析
 */

const jy901$ = new Rx.Subject()  // 发射JY901数据，这里它只发送数据，不观测数据，所以命名后边加上Observable
exports.jy901$ = jy901$
let jy901Info = ''
let jy901Completed = 0b000  // 数据完成的状态，0b111时表示获取完成
/**
 * 解析JY901模块回传内容，并转换成本系统的数据格式，发送可观测数据流
 *
 * 其回传内容为以下格式：
 * 数据头  数据包类型           数据                    校验和
 * 0x55     0x5*       short x  4 （8 bytes）    前10个字节相加(byte)      总共11个字节
 * 数据包为0x50的为片上时间回传，有不同的格式：
 * 0x55     0x50      YY MM DD hh mm ss msl msh        校验和             总共11个字节
 *
 * 片上时间：0x55     0x50      YY MM DD hh mm ss msl msh        校验和
 * YY:20YY年  MM:月  DD:日   hh:时  mm:分  ss:秒  msl msh:毫秒(short)
 *
 * 加速度：0x55 0x51 AxL AxH AyL AyH AzL AzH TL TH SUM  ( TL TH 是温度)
 *
 * 角速度：0x55 0x52 wxL wxH wyL wyH wzL wzH TL TH SUM ( TL TH 是温度)
 *
 * 角度：0x55 0x53 RollL RollH PitchL PitchH YawL YawH TL TH SUM ( TL TH 是温度)
 *
 * 磁场：0x55 0x54 HxL HxH HyL HyH HzL HzH TL TH SUM ( TL TH 是温度)
 *
 * 解析过程中填充jy901Info字符串，最终构造0x81数据包：
 * 0x66  0xaa     0x##      0x81    10 个 字符串   0x##     0xfc
 * JY901数据（3个加速度，3个角速度，3个角度[pitch、roll、yaw]，温度）
 */
exports.parseJY901Packet = function (packet) {
  let shortBuf = Buffer.alloc(2)
  switch (packet[1]) {
    case '\x51':  // 加速度
    {
      shortBuf.write(packet.substr(2, 2), 'binary')
      const ax = (shortBuf.readInt16LE(0) / 32768.0 * 16)
      shortBuf.write(packet.substr(4, 2), 'binary')
      const ay = (shortBuf.readInt16LE(0) / 32768.0 * 16)
      shortBuf.write(packet.substr(6, 2), 'binary')
      const az = (shortBuf.readInt16LE(0) / 32768.0 * 16)

      jy901Info += `${ax} ${ay} ${az} `

      jy901Completed |= 0b100
    }
      break
    case '\x52':  // 角速度
    {
      if ((jy901Completed & 0b100) !== 0b100) {
        // 如果接收该信息前未接收到加速度，由于传感器是按顺序发送的故说明本轮接收不完整，抛弃此次数据
        jy901Info = ''
        jy901Completed = 0b000

      } else {

        shortBuf.write(packet.substr(2, 2), 'binary')
        const wx = (shortBuf.readInt16LE(0) / 32768.0 * 2000)
        shortBuf.write(packet.substr(4, 2), 'binary')
        const wy = (shortBuf.readInt16LE(0) / 32768.0 * 2000)
        shortBuf.write(packet.substr(6, 2), 'binary')
        const wz = (shortBuf.readInt16LE(0) / 32768.0 * 2000)

        jy901Info += `${wx} ${wy} ${wz} `
        jy901Completed |= 0b010
      }

    }
      break
    case '\x53':  // 角度
    {
      if ((jy901Completed & 0b100) !== 0b100 || (jy901Completed & 0b010) !== 0b010) {
        // 如果接收该信息前未接收到加速度和角速度，由于传感器是按顺序发送的故说明本轮接收不完整，抛弃此次数据
        jy901Info = ''
        jy901Completed = 0b000
      } else {

        shortBuf.write(packet.substr(2, 2), 'binary')
        const pitch = (shortBuf.readInt16LE(0) / 32768.0 * 180)
        shortBuf.write(packet.substr(4, 2), 'binary')
        const roll = (shortBuf.readInt16LE(0) / 32768.0 * 180)
        shortBuf.write(packet.substr(6, 2), 'binary')
        const yaw = (shortBuf.readInt16LE(0) / 32768.0 * 180)
        // 取温度
        shortBuf.write(packet.substr(8, 2), 'binary')
        const temp = (shortBuf.readInt16LE(0) / 100.0)

        jy901Info += `${pitch} ${roll} ${yaw} ${temp}`

        jy901Completed |= 0b001
      }
    }
      break
    default:
      break
  }

  if (jy901Completed === 0b111) {  // 获取完毕，发送流
    // 0x66  0xaa     0x##      0x81    10 个 字符串       0x##     0xfc
    // JY901数据（3个加速度，3个角速度，3个角度[pitch、roll、yaw]，温度）

    const dataToCheck = ByteToString(jy901Info.length) + '\x81' + jy901Info
    jy901$.next(HEAD1 + HEAD2 + dataToCheck + ByteToString(crc8(dataToCheck)) + END)  // 发送可观测流

    // 复位
    jy901Info = ''
    jy901Completed = 0b000

  }

}


/**
 * arduino 串口信息解析
 */

/**
 * 发射arduino串口通用数据
 * @type {Subject}
 */
const arduino$ = new Rx.Subject()
exports.arduino$ = arduino$
let arduinoInfo = ''
let arduinoCompleted = 0b000  // 数据完成的状态，0b111时表示获取完成
/**
 * 解析arduino从串口回传的内容，并转换成本系统的数据格式，发送可观测数据流
 *
 * 其回传内容为以下格式：
 * 数据头  数据包类型           数据                    校验和
 * 0x55     0x5*             8  byte                  1  byte
 *
 * DHT11（温湿度）：0x55     0x50      HUMI TEMP 0x0 0x0 0x0 0x0 0x0 0x0       校验和
 * HUMI是湿度，TEMP是温度，0x0 表示留空
 *
 * 红外：0x55 0x52 S 0x0 0x0 0x0 0x0 0x0 0x0 0x0 SUM
 * S 第X位表示第X个红外传感器的值，0x0 表示留空
 *
 * 超声波：0x55     0x51      DL0 DH0 DL1 DH1 DL2 DH2 DL3 DH3        校验和
 * DLX DHX 表示第X个超声波传感器获得的距离值，以小端short方式存储
 *
 * 霍尔测速：0x55 0x53 left_count right_count 0x0 0x0 0x0 0x0 0x0 0x0       校验和
 *
 * 解析过程中填充arduinoInfo字符串，最终构造0x82数据包：
 * 0x66  0xaa     0x##      0x82    14 个字符串     0x##     0xfc
 * 传感器数据（湿度、温度、8个红外（1表示不正常、0正常），4个超声波）
 */
exports.parseArduinoPacket = function (packet) {
  let shortBuf = Buffer.alloc(2)
  switch (packet[1]) {
    case '\x50': {  // 温湿度
      shortBuf.write(packet.substr(2, 1), 'binary')
      const HUMI = shortBuf.readUInt8(0)
      shortBuf.write(packet.substr(3, 1), 'binary')
      const TEMP = shortBuf.readUInt8(0)

      arduinoInfo = `${HUMI} ${TEMP} `
      arduinoCompleted |= 0b100
    }
      break
    case '\x51': {  // 红外
      if ((arduinoCompleted & 0b100) !== 0b100) {
        // 如果接收该信息前未接收到温湿度，由于传感器是按顺序发送的故说明本轮接收不完整，抛弃此次数据
        arduinoInfo = ''
        arduinoCompleted = 0b000

      } else {
        // 只有一个字节有用，这里使用shortBuf即可读取出数字
        shortBuf.write(packet.substr(2, 1), 'binary')
        const d = shortBuf.readUInt8(0)
        /**
         * arduino传来的这个字节每一位对应 D0 ... D7
         * 红外避障传感器的输入值，D0, D1, D2, D3为向下的传感器，
         * D4, D5, D6, D7为向前，排列顺序为以左车头开始顺时针，
         * 数值为0说明距离小于阈值（led灯亮），为1说明距离大于阈值。
         * D0, D1, D2, D3期望为0，指示没有远离地面（以防移动后坠落），
         * D4, D5, D6, D7期望为1，指示前方没有障碍物（以防移动后碰撞）。
         *
         * 这里进行信息解析，1表示不正常，0表示正常
         */

        let tmp = 0b100000000
        for (let i = 0; i < 4; i++) {  // D0, D1, D2, D3
          tmp /= 2
          if ((d & tmp) != 0)  // 不为0，说明远离地面，不正常
            arduinoInfo += `1 `
          else
            arduinoInfo += `0 `
        }
        for (let i = 0; i < 4; i++) {  // D4, D5, D6, D7
          tmp /= 2
          if ((d & tmp) != 0)  // 不为0，说明前方没有障碍物，正常
            arduinoInfo += `0 `
          else
            arduinoInfo += `1 `
        }

        // arduinoCompleted |= 0b010
        arduinoCompleted |= 0b011  // skip \x52
      }
    }
      break
    case '\x52': {  // 超声波
      if ((arduinoCompleted & 0b100) !== 0b100 || (arduinoCompleted & 0b010) !== 0b010) {
        // 如果接收该信息前未接收到温湿度和红外，由于传感器是按顺序发送的故说明本轮接收不完整，抛弃此次数据
        arduinoInfo = ''
        arduinoCompleted = 0b000

      } else {

        shortBuf.write(packet.substr(2, 2), 'binary')
        const d0 = shortBuf.readInt16LE(0)
        shortBuf.write(packet.substr(4, 2), 'binary')
        const d1 = shortBuf.readInt16LE(0)
        shortBuf.write(packet.substr(6, 2), 'binary')
        const d2 = shortBuf.readInt16LE(0)
        shortBuf.write(packet.substr(8, 2), 'binary')
        const d3 = shortBuf.readInt16LE(0)

        arduinoInfo += `${d0} ${d1} ${d2} ${d3}`
        arduinoCompleted |= 0b001
      }
    }
      break
    case '\x53': {  // 测速，这里需要根据控制电机的命令的正负来判断前进后退，所以此处作为另一种类型的数据发送出去
      shortBuf.write(packet.substr(2, 2), 'binary')
      let leftCount = shortBuf.readInt16LE(0)
      shortBuf.write(packet.substr(4, 2), 'binary')
      let rightCount = shortBuf.readInt16LE(0)

      leftCount = speedL < 0 ? -leftCount : leftCount
      rightCount = speedR < 0 ? -rightCount : rightCount

      // 发送流
      const infos = `${leftCount} ${rightCount}`
      const dataToCheck = ByteToString(infos.length) + '\x83' + infos
      arduino$.next(HEAD1 + HEAD2 + dataToCheck + ByteToString(crc8(dataToCheck)) + END)
    }
      break
    default:
      break
  }

  if (arduinoCompleted == 0b111) {  // 获取完毕，发送流
    // 0x66  0xaa     0x12      0x81    9 short       0x##     0xfc
    const dataToCheck = ByteToString(arduinoInfo.length) + '\x82' + arduinoInfo
    arduino$.next(HEAD1 + HEAD2 + dataToCheck + ByteToString(crc8(dataToCheck)) + END)

    // 复位
    arduinoInfo = ''
    arduinoCompleted = 0b000

  }

}






