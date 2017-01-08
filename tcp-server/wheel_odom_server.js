/**
 * Created by Alvin Liu on 2016/5/3.
 * 本进程监听端口61616，广播轮式里程计数据给客户端
 *
 */

const net = require('net')
const Rx = require('rxjs/Rx')
const logger = require('./log')
const loginRexExp = /#.+:.+#/  // #guess:666666#

const wheelOdomProxy = new Rx.Subject()  // 一个代理，接收到python发来的数据后，立马广播给其他客户端

const wheelOdomServer = net.createServer((client) => {

  let user = 'nobody'
  let buf = ''  // 分配一个字符串buffer
  let subscription = null  // 每个客户端都会有一个订阅，在客户端断开时应该关闭它

  let address = `${client.remoteAddress}:${client.remotePort}`

  logger.info(`[wheel_odom_server]: client from ${address} connected`)

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
            if (username === 'python') {  // python进程连接进来

              logger.info(`[info server]: python client connected`)
              user = 'python'
              client.write('Now you can send data to me!\r\n')

              /**
               * ！！！生产数据过程
               * python进程登陆成功后会开始持续发送数据流，此时可以开始让infoProxy订阅python client的事件可观测对象
               * 这样订阅了wheelOdomProxy这个subject（此时作为Observer）的客户端就可以接收infoProxy广播的python数据了
               */
              subscription = Rx.Observable.fromEvent(client, 'data').subscribe(wheelOdomProxy)

            } else {  // 其他用户登录成功
              user = username

              /**
               * ！！！消费数据过程
               * 登陆成功后就可以开始订阅可观测对象，一有数据流则发送给对端socket（消费者客户端），
               * 我们的infoProxy以代理身份将发送python发来的数据广播给客户端
               */

              subscription = wheelOdomProxy.subscribe(
                (data) => {  // wheelOdomProxy从python进程获取到数据后传递到这里

                  client.write(data, 'binary')  // 广播给需要接收数据的客户端
                },
                (err) => {
                  logger.error(`[wheel_odom_server]: ${err}`)
                }
              )

            }

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
      logger.info(`[wheel_odom_server]: client disconnected.[user: ${user}, address: ${address}]`)
    })
    .on('error', (err) => {
      subscription && subscription.unsubscribe()
      logger.error(`[wheel_odom_server]: client error: ${err}.[user: ${user}, address: ${address}]`)
    })


})

wheelOdomServer.on('error', (err) => {
  logger.error(`[wheel_odom_server]: ${err}`)
})

wheelOdomServer.listen(61616, '0.0.0.0', () => {
  logger.info('wheel odom server bound')
})

exports.wheelOdomServer = wheelOdomServer
