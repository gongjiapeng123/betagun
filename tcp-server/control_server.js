/**
 * Created by Alvin Liu on 2016/5/3.
 * 在端口 61611 启动TCP服务器，客户端会与本服务器进行通信，以控制小车 （仅有一个客户端）
 */


const net = require('net')
const directive = require('./directive')
const logger = require('./log')

const CMDNUMBER_PER_EVENTDATA = 10  // 一次data事件要执行的命令
const cmdRegExp = /\x66\xaa.{3,}\xfc/g
const loginRexExp = /#.+:.+#/  // #guess:666666#

let pyConnected = false  // python进程客户端是否已经连接上来
let hasAdmin = false  // 是否已经有admin登入进来

/**
 * TCP控制服务器
 */
const controlServer = net.createServer((client) => {  // 当每一个client连接进来时
  // 'connection' listener

  let user = 'nobody'
  let buf = ''  // 每个客户端分配一个字符串buffer
  let cmdLines = []  // 存储每个客户端的当前还未执行的命令
  const address = `${client.remoteAddress}:${client.remotePort}`

  logger.info(`[control server]: client from ${address} connected`)
  client.write('Info: Welcome\r\n')

  // client socket的事件监听
  client
    .setEncoding('binary')  // 使用二进制字符串处理命令
    .on('data', (data) => {  // 接收到客户端发来的数据

      /**
       * step1：登录并订阅可观测流
       * 未以admin身份登陆的client，服务器只向其发送数据，不接收client的命令
       */
      if (user === 'nobody') {
        // 接收client的登陆信息
        buf += data

        const res = buf.match(loginRexExp)
        // res：  [匹配到的字符串，匹配的字符串的起始字符的id，正则表达式正在检测的输入字符串]
        if (res) {  // 有登录信息则获取信息并清空buf，否则无操作直到有登陆信息
          buf = ''
          // #guess:666666#  ==>  guess:666666  ==>  ['guess', '666666']
          const tmp = res[0].slice(1, -1).split(':')
          const username = tmp[0]
          const password = tmp[1]
          if (require('./check_user')(username, password)) {  // 登陆成功
            if (username == 'python') {
              pyConnected = true
              user = 'python'
              logger.info(`[control server]: python client connected`)

            } else if (username == 'admin') {
              if (hasAdmin) {  // 如果已经有客户端以admin身份登入则不允许登入
                client.write('Error: I already have a master!\r\n')
                return
              }

              // 还未有admin登陆，则本次以admin身份登录成功
              hasAdmin = true
              user = 'admin'
              client.write('Info: Waiting your order, my master!\r\n')

            } else {  // 普通用户登录成功
              user = username
              client.write('Info: Welcome your visit, my friend!\r\n')
            }

          } else {
            client.write('Error: wrong username or password!\r\n')
          }
        } else {
          if (buf.length > 8192) {  // 登录前buf数据太多
            buf = ''
          }
        }

        // 因为没有登录，所以不接收client的命令，直接return
        return
      }

      /**
       * step2：接收命令并执行命令（这里只有python 或 admin身份的client会执行）
       * ！！！client以admin登录后才会执行以下程序
       * 客户发送过来的命令必须是以下结构：
       * 头             栈长度     命令字   数据          校验和      结束字节
       * 0x66  0xaa     0x08      0x02     "+100-100"     0x##      0xfc
       */
      if (user === 'admin') {
        buf += data  // 存入buf中

        let lastIndex = 0  // 保存当前data事件中，对buf进行正则匹配循环时的下次要进行正则匹配的起始字符所在的id
        let matchCmds = []  // 保存当前data事件中，对buf进行正则匹配所匹配到的命令行

        // 开始分割命令并存储
        while (true) {
          let matchItem = cmdRegExp.exec(buf)
          // matchItem：  [匹配到的字符串，匹配的字符串的起始字符的id，正则表达式正在检测的输入字符串]
          if (matchItem) {
            matchCmds.push(matchItem[0])
            lastIndex = cmdRegExp.lastIndex
          } else {
            // 匹配完了，此时matchItem = null，且reg.lastIndex会变回0，
            // 所以用一个变量lastIndex来保存最后一个匹配成功后的起始字符id
            break
          }
        }

        if (matchCmds) {
          cmdLines = cmdLines.concat(matchCmds)  // 追加至未完成的命令列表中
          buf = buf.slice(lastIndex)  // 将已经进行正则匹配过的字符串剪掉
        }

        // 一次data事件中需要执行的命令，一次性不要执行太多次
        // 若cmds个数不大于CMDNUMBER_PER_EVENTDATA，cmds会被全部取出
        const todos = cmdLines.splice(0, CMDNUMBER_PER_EVENTDATA)
        if (todos.length > 0) {
          // 遍历每条命令
          todos.forEach((cmdLine) => {
            const cmd = directive.parseCommand(cmdLine)
            directive.executeCommand(cmd)
          })
        }
      }
    })
    .on('end', () => {  // 连接结束时
      if (user === 'python') {
        pyConnected = false
      }
      else if (user === 'admin') {
        hasAdmin = false
      }
      logger.info(`[control server]: client disconnected.[user: ${user}, address: ${address}]`)
    })
    .on('error', (err) => {  // 连接错误时
      if (user == 'python') {
        pyConnected = false
      }
      else if (user == 'admin') {
        hasAdmin = false
      }
      logger.error(`[control server]: client error: ${err}.[user: ${user}, address: ${address}]`)
    })


})

controlServer.on('error', (err) => {
  logger.error(`[control server]: ${err}`)
})

controlServer.listen(61611, '0.0.0.0', () => {
  logger.info('control server bound')
})

exports.controlServer = controlServer
