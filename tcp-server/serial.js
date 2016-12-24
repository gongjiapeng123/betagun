/**
 * Created by Alvin Liu on 2016/4/29.
 * 对下位机通信的串口的设置
 */

const SerialPort = require('serialport')
const directive = require('./directive')
const logger = require('./log')

// 获取环境变量的串口号，当不同设备运行时，分配名为
// BETAGUN_MOTOR_COMPORT、BETAGUN_JY901_COMPORT、BETAGUN_SENSORS_COMPORT的环境变量即可

/**
 * 电机驱动板串口，发送电机控制指令
 */

let motorPortName = ''
if (process && process.env && process.env.BETAGUN_MOTOR_COMPORT) {
  motorPortName = process.env.BETAGUN_MOTOR_COMPORT
}

const motorPort = new SerialPort(motorPortName, {
  baudRate: 9600,
  dataBits: 8,
  stopBits: 1,
  parity: 'none',
  parser: SerialPort.parsers.readline('\r\n')
})
  .on('open', (err) => {  // 订阅事件
    if (err) {
      logger.error(err)
    } else {
      logger.info('[motor port]: opened!')
    }

  })
  .on('close', () => {
    logger.info('[motor port]: closed!')

  })
  .on('error', (err) => {
    logger.error(`[motor port]: error: ${err}`)

  })
  .on('disconnect', (err) => {
    logger.info(`[motor port]: disconnect: ${err}`)

  })

exports.motorPort = motorPort

/**
 * JY901九轴传感器串口，接收传感器数据
 */

let jy901PortName = ''
if (process && process.env && process.env.BETAGUN_JY901_COMPORT) {
  jy901PortName = process.env.BETAGUN_JY901_COMPORT
}

/**
 * 串口的解析接收数据的方法
 * @returns {Function}
 * 该方法每当有二进制数据从串口传入都会触发一次。直到没有0x55在buffer中
 * 串口回传的数据包以0x55开头，但是并没有结束字节。这里由于其回传数据包中在起始字节0x55后面表示数据包类型的
 * 字节是0x5*，有0x50 & 0x5* = 0x50，所以，可以根据这个特点来提取数据包
 */
function packetParser () {
  let buffer = ''
  return (emitter, data) => {
    // 保存数据
    buffer += data.toString('binary')
    // 解析
    let index = buffer.indexOf('\x55')
    while (index > -1) {  // 如果当前已经收到buffer中第一个数据包的起始字节0x55
      if (index == buffer.length - 1) break  // 如果0x55后面没有数据直接退出循环
      if ((0x50 & Buffer.from(buffer[index + 1], 'binary').readUInt8(0)) != 0x50)  // 不满足数据包格式
        buffer = buffer.slice(index + 1)  // 丢弃当前buffer第一个数据包之前无用的数据
      else {  // 满足数据包格式
        buffer = buffer.slice(index)  // 丢弃当前buffer第一个数据包之前无用的数据
        if (buffer.length > 11) {  // 如果长度超过11，说明buffer中第一个数据包可以获取
          let packet = buffer.slice(0, 11)
          buffer = buffer.slice(11)

          // 检查校验和，为前10个字节相加，int8方式存储，所以这里模拟截断
          let sum = 0
          let bytesArray = Buffer.from(packet, 'binary')
          for (let i = 0; i < 10; i++) {
            sum += bytesArray[i]

            if (sum > 0x7F)  // 上溢出
              sum -= 256
            if (sum < -0x80)  // 下溢出
              sum += 256
          }
          let pSum = bytesArray[10]  // 数据包中存储的校验和
          if (pSum > 0x7F)  // 上溢出
            pSum -= 256
          if (pSum < -0x80)  // 下溢出
            pSum += 256

          if (sum == pSum) {  // 满足校验和
            emitter.emit('data', packet)

          }

          // 继续下一回循环
          index = buffer.indexOf('\x55')

        } else  // 否则获取足够数据后再进行本次函数
          break
      }
    }

  }
}

const jy901Port = new SerialPort(jy901PortName, {
  baudRate: 115200,
  dataBits: 8,
  stopBits: 1,
  parity: 'none',
  parser: packetParser()
})
  .on('open', (err) => {  // 订阅事件
    if (err) {
      logger.error(err)
    } else {
      logger.info('[jy901 port]: opened')
    }

  })
  .on('data', (packet) => {  // 收到一条数据
    directive.parseJY901Packet(packet)

  })
  .on('close', () => {
    logger.info('[jy901 port]: closed!')

  })
  .on('error', (err) => {
    logger.error(`[jy901 port]:  error: ${err}`)

  })
  .on('disconnect', (err) => {
    logger.info(`[jy901 port]:  disconnect: ${err}`)

  })

exports.jy901Port = jy901Port


/**
 * arduino串口，接收传感器数据
 */

let arduinoPortName = ''
if (process && process.env && process.env.BETAGUN_ARDUINO_COMPORT) {
  arduinoPortName = process.env.BETAGUN_ARDUINO_COMPORT
}

const arduinoPort = new SerialPort(arduinoPortName, {
  baudRate: 115200,
  dataBits: 8,
  stopBits: 1,
  parity: 'none',
  parser: packetParser()
})
  .on('open', (err) => {  // 订阅事件
    if (err) {
      logger.error(err)
    } else {
      logger.info('[arduino port]: opened')
    }

  })
  .on('data', (packet) => {  // 收到一条数据
    directive.parseArduinoPacket(packet)

  })
  .on('close', () => {
    logger.info('[arduino port]:  closed!')

  })
  .on('error', (err) => {
    logger.error(`[arduino port]: error: ${err}`)

  })
  .on('disconnect', (err) => {
    logger.info(`[arduino port]:  disconnect: ${err}`)

  })

exports.arduinoPort = arduinoPort

