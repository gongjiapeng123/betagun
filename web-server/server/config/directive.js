/**
 * Created by alvin.liu on 2016/4/29.
 * 构造命令发送给小车tcp服务器，和解析TCP服务器传来的信息（此处在web服务器解析，也可以在浏览器客户端解析）
 */


import * as observables from './observables'
import crc from 'crc'
import log4js from 'log4js'
const logger = log4js.getLogger('default')

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
 * 生成控制电机命令
 * 头             栈长度     命令字   数据          校验和      结束字节
 * 0x66  0xaa     0x08      0x03     "+100-100"     0x##      0xfc
 * @param motorLeftSpeed {number}
 * @param motorRightSpeed {number}
 * @returns {string} binary
 */
export function makeControlMotorCommand (motorLeftSpeed, motorRightSpeed) {
  let data = ''
  let temp = Math.abs(motorLeftSpeed).toString()
  if (temp.length == 1)
    temp = '00' + temp
  else if (temp.length == 2)
    temp = '0' + temp
  if (motorLeftSpeed >= 0) {
    data = '+' + temp
  }
  else
    data = '-' + temp

  temp = Math.abs(motorRightSpeed).toString()
  if (temp.length == 1)
    temp = '00' + temp
  else if (temp.length == 2)
    temp = '0' + temp
  if (motorRightSpeed >= 0)
    data += '+' + temp
  else
    data += '-' + temp
  let dataToCheck = '\x08' + '\x03' + data
  return HEAD1 + HEAD2 + dataToCheck + ByteToString(crc.crc8(dataToCheck)) + END
}

/**********************************************************************************************************************
 以下是解析
 ***********************************************************************************************************************/

const THRESHOLD_DETECT_DATA_LEN = 8192  // 当buf剩下的数据大于该值时，多次执行数据检测

/**
 * 对从服务器61614端口获取到的数据进行解析，获取图像
 */


let imageBuf = ''
let imageLen = 0  // 图像长度
let headID = 0  // 记录一副图片头0x66在imageBuf中的index
let startID = 0  // 起始字符的id
let endID = 0  // 结束字符的id

/**
 * 解析图像数据的状态，这里不能使用正则表达式，因为随着数据增大，本函数执行的次数也会越来越多，越到后面执行正则查找花的时间越多
 * 0：还未发现0x66；1：已发现0x66；2：已发现0x66 0xaa；3：已接收到图像长度数据（16字节的字符串）并已获得数据字0x80；
 * 4：已接收完图像数据并获得0x00
 * @type {number}
 */
let imageState = 0


/**
 * 不断接收数据，解析出图像并发射可观测流
 *
 * ！！！注意，本函数是在socket接收到图像数据时才执行的，由于socket发送数据是一个chunk一个chunk发送过来的，（因为传输图像，chunk比较大）
 * 所以触发本次函数的次数跟这些chunk的个数一致，如果本次执行只运行一次switch检测,将导致buf越来越大，且解析跟不上，
 * （已踩坑，客户端显示慢了几十秒的图片，后来内存溢出= =）所以需要在上游扩流，让本函数触发的次数足够多（本函数作为rx的map操作函数的参数）
 * 或者在此处多次发射可观测流（本函数操作Observable），此处使用后者
 *
 * 头（2字节）    栈长度（16字节，字符串：'4545171         '，表示该图片的大小）  数据字   数据（图片数据）  校验和  结束字节
 * 0x66  0xaa     0x08                                                      0x80    "5fdfasdgag"    0x00   0xfc
 * @param data
 */
export function parseImage (data) {
  /**
   * 转换二进制字符串方式，这里要求传进来的数据经过toString('binary')
   */
  imageBuf += data

  while (imageBuf.length > THRESHOLD_DETECT_DATA_LEN) {
    switch (imageState) {
      case 0:  // 还未发现0x66
      {
        let res = imageBuf.indexOf('\x66')
        if (res > -1) {
          headID = res
          imageState = 1
        }
      }
        break
      case 1:  // 已发现0x66，进入此处的imageBuf的长度至少为(headID + 1) + (1)
      {
        let res = imageBuf.slice(headID + 1).indexOf('\xaa')
        if (res == 0) {  // 截取了buf后0xaa应该位于0索引
          imageState = 2

        } else {  // 不匹配，清除已经检测过的buf
          imageState = 0
          if (res > 0 && imageBuf.length > res + 1)  // 如果检测到\xaa但index不为0且后面还有数据，则保留那些没有检测过的数据
            imageBuf = imageBuf.slice(res + 1)
          else {  // 这时直接清空buf即可（丢弃无效的数据）
            imageBuf = ''
          }
        }
      }
        break
      case 2:  // 已发现0x66 0xaa，进入此处的imageBuf的长度至少为(headID + 1) + (2)
      {
        if (imageBuf.length > headID + 1 + 2 + 16) {  // 0x66  0xaa 之后接收了17字节(包括)以上的数据
          let res = imageBuf.slice(headID + 18).indexOf('\x80')  // 截取再查找，以防表示图像字节数的16字节中有\x80

          if (res == 0) {  // 截取了buf后0x80应该位于0索引
            imageLen = parseInt(imageBuf.substr(headID + 2, 16).trim())
            startID = headID + 19  // 2 + 16 + 1(0x80)
            endID = startID + imageLen - 1 // 计算图片的起始字符的id和结束字符的id
            imageState = 3

          } else {  // 不匹配，清除已经检测过的buf
            imageState = 0
            if (res > 0 && imageBuf.length > res + 1)  // 如果检测到\x80但index不为0且后面还有数据，则保留那些没有检测过的数据
              imageBuf = imageBuf.slice(res + 1)
            else {  // 这时直接清空buf即可（丢弃无效的数据）
              imageBuf = ''
            }
          }

        }
      }
        break
      case 3:  // 已接收到图像长度数据（16字节的字符串）并获得数据字0x80，进入此处的imageBuf的长度至少为(headID + 1) + (2 + 16 + 1)
      {
        if (imageBuf.length > endID + 1) {  // 接收到了足够的图像数据

          if (imageBuf[endID + 1] == '\x00')  // 此时图像数据最后一位字符的后一位正好是\x00，说明获取图像数据成功
            imageState = 4
          else {  // 不匹配，清除已经检测过的buf
            imageState = 0
            if (imageBuf.length > endID + 2)  // 如果(endID + 1)后面还有其他数据，则保留那些没有检测过的数据
              imageBuf = imageBuf.slice(endID + 2)
            else {  // 这时直接清空buf即可（丢弃无效的数据）
              imageBuf = ''
            }
          }
        } else {  // 此时buf中的数据不足以解析完整的图片，跳出循环以获取更多的数据后再进入本状态进行检测
          return
        }
      }
        break
      case 4:  // 已接收完图像数据并获得0x00，进入此处的imageBuf的长度至少为(endID + 1) + 2 ，其中的2是0x00 0xfc
      {
        let res = imageBuf.slice(endID + 2).indexOf('\xfc')
        if (res == 0) {  // 截取了buf后0xfc应该位于0索引，这时获取图片完成

          // console.log('get a image!')
          // console.log(++imgCount)
          observables.image$.next(imageBuf.slice(startID, endID + 1))  // 解析图片完成，发送流

          imageState = 0  // 不管检测是否成功都应该回复状态，并清空之前判断过的buf

          if (imageBuf.length > endID + 3)  // 如果在\xfc后面还有数据
            imageBuf = imageBuf.slice(endID + 3)
          else {  // 这时直接清空buf即可
            imageBuf = ''
          }


        } else { // 因为紧接着是\x00的应该是\xfc，找不到则说明数据错误
          imageState = 0

          if (imageBuf.length > endID + 3)  // 如果在(endID + 2)后面还有数据
            imageBuf = imageBuf.slice(endID + 3)
          else {  // 这时直接清空buf即可
            imageBuf = ''
          }
        }

      }
        break

    }


  }

  /**
   * buffer方式
   */

  // if (imageBuf) {
  //   imageBuf = Buffer.concat([imageBuf, data])
  // } else {  // 初始化buf
  //   imageBuf = Buffer.from(data)
  // }
  //
  // while (imageBuf.length > THRESHOLD_DETECT_DATA_LEN) {
  //   switch (imageState) {
  //     case 0:  // 还未发现0x66
  //     {
  //       let res = imageBuf.indexOf(102)  // 0x66
  //       if (res > -1) {
  //         headID = res
  //         imageState = 1
  //       }
  //     }
  //       break
  //     case 1:  // 已发现0x66，进入此处的imageBuf的长度至少为(headID + 1) + (1)
  //     {
  //       let res = imageBuf.slice(headID + 1).indexOf(170)  // 0xaa
  //       if (res == 0) {  // 截取了buf后0xaa应该位于0索引
  //         imageState = 2
  //
  //       } else {  // 不匹配，清除已经检测过的buf
  //         imageState = 0
  //         if (res > 0 && imageBuf.length > res + 1)  // 如果检测到\xaa但index不为0且后面还有数据，则保留那些没有检测过的数据
  //           imageBuf = imageBuf.slice(res + 1)
  //         else {  // 这时直接清空buf即可（丢弃无效的数据）
  //           imageBuf = null
  //         }
  //       }
  //     }
  //       break
  //     case 2:  // 已发现0x66 0xaa，进入此处的imageBuf的长度至少为(headID + 1) + (2)
  //     {
  //       if (imageBuf.length > headID + 1 + 2 + 16) {  // 0x66  0xaa 之后接收了17字节(包括)以上的数据
  //         let res = imageBuf.slice(headID + 18).indexOf(128)  // 截取再查找，以防表示图像字节数的16字节中有\x80
  //
  //         if (res == 0) {  // 截取了buf后0x80应该位于0索引
  //           imageLen = parseInt(imageBuf.slice(headID + 2, headID + 2 + 16).toString().trim())
  //           startID = headID + 19  // 2 + 16 + 1(0x80)
  //           endID = startID + imageLen - 1 // 计算图片的起始字符的id和结束字符的id
  //           imageState = 3
  //
  //         } else {  // 不匹配，清除已经检测过的buf
  //           imageState = 0
  //           if (res > 0 && imageBuf.length > res + 1)  // 如果检测到\x80但index不为0且后面还有数据，则保留那些没有检测过的数据
  //             imageBuf = imageBuf.slice(res + 1)
  //           else {  // 这时直接清空buf即可（丢弃无效的数据）
  //             imageBuf = null
  //           }
  //         }
  //
  //       }
  //     }
  //       break
  //     case 3:  // 已接收到图像长度数据（16字节的字符串）并获得数据字0x80，进入此处的imageBuf的长度至少为(headID + 1) + (2 + 16 + 1)
  //     {
  //       if (imageBuf.length > endID + 1) {  // 接收到了足够的图像数据
  //
  //         if (imageBuf[endID + 1] == 0)  // 此时图像数据最后一位字符的后一位正好是\x00，说明获取图像数据成功
  //           imageState = 4
  //         else {  // 不匹配，清除已经检测过的buf
  //           imageState = 0
  //           if (imageBuf.length > endID + 2)  // 如果(endID + 1)后面还有其他数据，则保留那些没有检测过的数据
  //             imageBuf = imageBuf.slice(endID + 2)
  //           else {  // 这时直接清空buf即可（丢弃无效的数据）
  //             imageBuf = null
  //           }
  //         }
  //       } else {  // 此时buf中的数据不足以解析完整的图片，跳出循环以获取更多的数据后再进入本状态进行检测
  //         return
  //       }
  //     }
  //       break
  //     case 4:  // 已接收完图像数据并获得0x00，进入此处的imageBuf的长度至少为(endID + 1) + 2 ，其中的2是0x00 0xfc
  //     {
  //       let res = imageBuf.slice(endID + 2).indexOf(252)  // 0xfc
  //       if (res == 0) {  // 截取了buf后0xfc应该位于0索引，这时获取图片完成
  //         // console.log('get a image!')
  //         console.log(++imgCount)
  //         observables.image$.next(imageBuf.slice(startID, endID + 1))  // 解析图片完成，发送流
  //
  //         imageState = 0  // 不管检测是否成功都应该回复状态，并清空之前判断过的buf
  //
  //         if (imageBuf.length > endID + 3)  // 如果在\xfc后面还有数据
  //           imageBuf = imageBuf.slice(endID + 3)
  //         else {  // 这时直接清空buf即可
  //           imageBuf = null
  //         }
  //
  //       } else { // 因为紧接着是\x00的应该是\xfc，找不到则说明数据错误
  //         imageState = 0
  //
  //         if (imageBuf.length > endID + 3)  // 如果在(endID + 2)后面还有数据
  //           imageBuf = imageBuf.slice(endID + 3)
  //         else {  // 这时直接清空buf即可
  //           imageBuf = null
  //         }
  //       }
  //
  //     }
  //       break
  //   }
  //
  // }

}


/**
 * 对从服务器61612、61613、61615端口获取到的数据进行解析，获取传感器信息或计算信息
 */


/**
 * 数据包的描述
 * @type {MapConstructor|Map}
 */
const DATAMAP = new Map([
  ['\x00', {description: '未能解析参数'}],
  ['\x81', {description: 'JY901数据（3个加速度，3个角速度，3个角度[pitch、roll、yaw]'}],
  ['\x82', {description: ''}]
])

/**
 * 数据包工厂函数
 * @param dataLine
 * @param dataID
 * @param data
 * @returns {DataPacket}
 * @constructor
 */
function DataPacketFactory (dataLine, dataID, data) {
  const dataInfo = DATAMAP.get(dataID)
  return new DataPacket(dataLine, dataID, dataInfo.description, data)
}

class DataPacket {
  constructor (dataLine, dataID, description, data) {
    this.dataLine = dataLine
    this.dataID = dataID
    this.description = description
    this.data = data
  }
}

/**
 * 校验数据包
 * @param dataLine
 * @returns {DataPacket}
 * @private
 */
function _check (dataLine) {
  let dataID = dataLine[3]
  const dataLen = Buffer.from(dataLine[2], 'binary')[0]
  const data = dataLine.substr(4, dataLen)

  let checkSum = dataLine.slice(-2, -1)
  let checkData = dataLine.slice(2, -2)  // 栈长度 + 命令字 + 数据

  // let b = Buffer.from(checkData, 'binary')

  if (ByteToString(crc.crc8(checkData)) != checkSum)  // 校验和不匹配
    dataID = '\x00'

  return DataPacketFactory(dataLine, dataID, data)

}

/**
 * 发射websocket事件
 * @param packet
 * @private
 */
function _emit (packet) {
  switch (packet.dataID) {
    case '\x81':  // jy901传感器数据
    {
      observables.jy901$.next(packet.data)
    }
      break
    case '\x82':  // arduino的传感器数据
    {
      observables.arduino$.next(packet.data)
    }
      break

    default:
      break

  }
}

let dataBuf = ''  // 缓存buf
const cmdRegExp = /\x66\xaa.{3,}\xfc/g

/**
 * 不断接收数据，解析出数据并发射可观测流，这里使用正则表达式来提取信息
 *
 * 头（2字节）    栈长度     数据字    数据             校验和    结束字节    意义
 * 0x66  0xaa    0xxx      0x81    "5fdfasdgag"      0xxx      0xfc      JY901数据（3个加速度，3个角速度，3个角度[pitch、roll、yaw]）
 *
 *
 * @param data
 */
export function parseData (data) {

  dataBuf += data

  let lastIndex = 0  // 保存本次调用中，对buf进行正则匹配循环时的下次要进行正则匹配的起始字符所在的id

  // 开始分割数据
  while (true) {
    let matchItem = cmdRegExp.exec(dataBuf)
    // matchItem返回：  [匹配到的字符串，匹配的字符串的起始字符的id，正则表达式正在检测的输入字符串]
    if (matchItem) {  // 匹配成功
      let packet = _check(matchItem[0])
      _emit(packet)

      lastIndex = cmdRegExp.lastIndex

    } else {
      // 匹配完了，此时matchItem = null，且reg.lastIndex会变回0，
      // 所以用一个变量lastIndex来保存最后一个匹配成功后的起始字符id
      dataBuf = dataBuf.slice(lastIndex)  // 将已经进行正则匹配过的字符串剪掉
      break
    }
  }


}