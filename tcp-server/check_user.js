/**
 * Created by Alvin Liu on 2016/5/4.
 *
 * 验证用户，小应用就不用数据库了，这里的user只有python、admin、guess
 * 三个，不同权限获取得到的信息不一样，python是python进程连接上来的登陆名，
 * 它发送的命令是权限最高的，admin只能有一个客户端可以登陆，它将获得
 * 图像等所有信息并且可以控制小车，guess登录只可以获得展示性的数据
 */


module.exports = function checkUser (username, password) {
  const users = new Map().set('python', 'gxnu').set('admin', 'gxnu').set('guess', '666666')
  return users.has(username) && users.get(username) === password
}