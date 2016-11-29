import passport from 'koa-passport'
import { Strategy as LocalStrategy } from 'passport-local'
import User from '../models/User'

/**
 * user.id序列化到session, 同时它将作为凭证存储在用户cookie中
 */
passport.serializeUser((user, done) => {
  done(null, user.id)
})

/**
 * 从session反序列化，参数为用户提交的sessionID，若存在则从数据库中查询user并存储于req.user中
 */
passport.deserializeUser((id, done) => {
  User.findOne(id, function (err, user) {
    done(err, user)
  })
})


passport.use(new LocalStrategy(
  {
    usernameField: 'username',
    passwordField: 'password',
  },
  async (username, password, done) => {
    // const criteria = (username.indexOf('@') === -1) ? {username: username} : {email: username}  // 提供多种用户类型
    const result = await User.verify(username, password)
    if (!!result) {
      done(null, result)
    } else {
      done(null, false)
    }
  })
)
