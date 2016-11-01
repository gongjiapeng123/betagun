/**
 * Created by Alvin Liu on 2016/5/3.
 * 本进程监听端口61613，广播arduino数据给客户端
 *
 */

const net = require('net')
const Rx = require('rxjs/Rx')

const loginRexExp = /#.+:.+#/  // #guess:666666#

const arduinoProxy = new Rx.Subject()
// 接收arduino串口信息 ！！！生产数据过程
require('./directive').arduinoObservable.subscribe(arduinoProxy)

const arduinoServer = net.createServer((client) => {

  let user = 'nobody'
  let buf = ''  // 分配一个字符串buffer
  let subscription = null  // 每个客户端都会有一个订阅，在客户端断开时应该关闭它

  let address = `${client.remoteAddress}:${client.remotePort}`

  console.log(`client from ${address} connected`)

  // client socket的事件监听
  client
    .on('data', (data) => {

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
            user = username

            /**
             * ！！！消费数据过程
             */
            subscription = arduinoProxy.subscribe(
              (data) => {
                client.write(data, 'binary')  // 广播给需要接收数据的客户端
              },
              (err) => {
                console.error(err)
              }
            )

          } else {
            client.write('Wrong username or password!\r\n')
          }
        }

      } else {  // 登录前buf数据太多
        if (buf.length > 8192) {
          buf = ''
        }
      }

    })
    .on('end', () => {  // 连接结束时
      subscription && subscription.unsubscribe()
      console.log(`client disconnected.[user: ${user}, address: ${address}]`)
    })
    .on('error', (err) => {
      subscription && subscription.unsubscribe()
      console.error(`client error: ${err}.[user: ${user}, address: ${address}]`)
    })


})

arduinoServer.on('error', (err) => {
  console.log(err);
})

arduinoServer.listen(61613, () => {
  console.log('arduino server bound')
})

exports.arduinoServer = arduinoServer
