/**
 * Created by Alvin Liu on 2016/5/3.
 * 本进程监听端口61615，广播运算数据给客户端
 *
 */

const net = require('net')
const Rx = require('rxjs/Rx')
const logger = require('./log')
const _ = require('lodash')
const loginRexExp = /#.+:.+#/  // #guess:666666#


// 记录对python进程数据的代理列表，给客户端订阅以广播数据，在client连接之前python进程应该连接完毕
// 即client连进来之前infoProxies的列表应该已经确定长度，否则新连接的python进程的数据对于之前的client将获取不到
const infoProxies = []

const infoServer = net.createServer((client) => {

  let user = 'nobody'
  let buf = ''  // 分配一个字符串buffer
  let subscriptions = []  // 每个客户端都会有多个订阅，在客户端断开时应该关闭它

  let address = `${client.remoteAddress}:${client.remotePort}`

  logger.info(`[info server]: client from ${address} connected`)

  // client socket的事件监听
  client
    .on('data', (data) => {
      // 1、接收到python进程发来的运算数据，一收到数据就立马发送可观测流到dataProxy
      // 2、客户端连接进来后将订阅运算数据可观测流

      /**
       * 登录以确认python进程的连接，这里除了python进程客户端，对于其他客户端，只接收登录信息，不接收其他数据
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
            if (username === 'python') {  // python进程连接进来

              logger.info(`[info server]: python client connected`)
              user = 'python'
              client.write('Now you can send data to me!\r\n')

              /**
               * ！！！生产数据过程
               * python进程登陆成功后会开始持续发送数据流，此时可以开始让infoProxy订阅python client的事件可观测对象
               * 这样订阅了infoProxy这个subject（此时作为Observer）的客户端就可以接收infoProxy广播的python数据了
               */
              const infoProxy = new Rx.Subject()
              infoProxies.push(infoProxy)
              const subscription = Rx.Observable.fromEvent(client, 'data').subscribe(infoProxy)
              subscriptions.push(subscription)

            } else {  // 其他用户登录成功
              user = username

              /**
               * ！！！消费数据过程
               * 登陆成功后就可以开始订阅可观测对象，一有数据流则发送给对端socket（消费者客户端），
               * 我们的infoProxy以代理身份将发送python发来的数据广播给客户端
               */

              subscriptions = infoProxies.map(infoProxy => {
                return infoProxy.subscribe(
                  (data) => {  // infoProxy从python进程获取到数据后传递到这里

                    client.write(data, 'binary')  // 广播给需要接收数据的客户端
                  },
                  (err) => {
                    logger.error(`[info server]: ${err}`)
                  }
                )
              })

            }

          } else {
            client.write('Wrong username or password!\r\n')
          }
        }

      } else {
        if (buf.length > 8192) {  // 登录前buf数据太多
          buf = ''
        }
      }

    })
    .on('end', () => {  // 连接结束时
      subscriptions.forEach(subscription => subscription && subscription.unsubscribe())
      logger.info(`[info server]: client disconnected.[user: ${user}, address: ${address}]`)
    })
    .on('error', (err) => {
      subscriptions.forEach(subscription => subscription && subscription.unsubscribe())
      logger.error(`[info server]: client error: ${err}.[user: ${user}, address: ${address}]`)
    })


})

infoServer.on('error', (err) => {
  logger.error(`[info server]: ${err}`)
})

infoServer.listen(61615, '0.0.0.0', () => {
  logger.info('info server bound')
})

exports.infoServer = infoServer
