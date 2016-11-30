/**
 * Created by Alvin Liu on 2016/5/3.
 * 本进程监听端口61614，接受python进程连接，获取python进程发送过来的图像数据，
 * 一接收到足够的数据就广播给客户端。
 *
 */


const net = require('net')
const Rx = require('rxjs/Rx')

const loginRexExp = /#.+:.+#/  // #guess:666666#
let pyConnected = false  // python进程客户端是否已经连接上来
const imageProxy = new Rx.Subject()  // 一个代理，接收到python发来的数据后，立马广播给其他客户端


/**
 * 接收python图像的TCP服务器
 */
const imageServer = net.createServer((client) => {
  // 'connection' listener

  // client的user，这里需要登录信息只是为了检测python进程的连接
  let user = 'nobody'
  let buf = ''  // 分配一个字符串buffer
  let subscription = null  // 每个客户端都会有一个订阅，在客户端断开时应该关闭它

  const address = `${client.remoteAddress}:${client.remotePort}`

  console.log(`client from ${address} connected`)

  // client socket的事件监听
  client
    .on('data', (data) => {
      // 1、接收到python进程发来的数据，一收到数据就立马发送可观测流
      // 2、接收到客户端的登录信息，登录成功后传递python进程生产的数据

      /**
       * 登录以确认python进程的连接，这里除了python进程客户端，对于其他客户端，只接收登录信息，不接收其他数据
       * python进程客户端的数据通过Rx库来生成可观测流以广播数据给其他客户端
       */
      if (user === 'nobody') {
        // 接收client的登陆信息
        buf += data
        let res = buf.match(loginRexExp)
        // res：  [匹配到的字符串，匹配的字符串的起始字符的id，正则表达式正在检测的输入字符串]

        if (res) {  // 有登录信息则获取信息并清空buf，否则无操作直到有登陆信息
          buf = ''
          let tmp = res[0].slice(1, -1).split(':')  // #guess:666666#  ==>  guess:666666  ==>  ['guess', '666666']
          let username = tmp[0]
          let password = tmp[1]
          if (require('./check_user')(username, password)) {  // 登陆成功
            if (username == 'python') {  // python进程连接61614端口

              console.log(`imageServer: python client connected`)
              pyConnected = true
              user = 'python'
              // client.write('Now you can send data to me!\r\n')

              /**
               * ！！！生产数据过程
               * 登陆成功后python进程会开始持续发送数据流，此时可以开始让imageProxy订阅python client的事件可观测对象
               * 这样imageProxy这个subject（此时作为Observer）就可以接收python发来的数据了
               */
              subscription = Rx.Observable.fromEvent(client, 'data').subscribe(imageProxy)

            } else {  // 其他用户登录成功
              user = username

              /**
               * ！！！消费数据过程
               * 登陆成功后就可以开始订阅可观测对象，一有数据流则发送给对端（客户端），这时我们的imageProxy广播数据
               */
              subscription = imageProxy.subscribe(
                (data) => {  // imageProxy从python进程获取到数据后传递到这里
                  client.write(data)
                },
                (err) => {  // 发生错误
                  console.error(err)
                }
              )

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
      if (user == 'python')
        pyConnected = false

      subscription && subscription.unsubscribe()
      console.log(`client disconnected.[user: ${user}, address: ${address}]`)
    })
    .on('error', (err) => {
      if (user == 'python')
        pyConnected = false

      subscription && subscription.unsubscribe()
      console.error(`client error: ${err}.[user: ${user}, address: ${address}]`)
    })


})

imageServer.on('error', (err) => {
  console.log(err)
})

imageServer.listen(61614, '0.0.0.0', () => {
  console.log('image server bound')
})

exports.imageServer = imageServer
