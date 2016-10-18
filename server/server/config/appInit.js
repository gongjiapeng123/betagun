
import compose from 'koa-compose'
import convert from 'koa-convert'
import cors from 'kcors'
import serve from 'koa-static'
import logger from 'koa-logger'
import mount from 'koa-mount'
import bodyParser from 'koa-bodyparser'
import session from 'koa-generic-session'

import './passport'  // 配置passport
import passport from 'koa-passport'

import env from './env'

export default function appInit (app) {

  app.name = 'server'
  app.proxy = true
  app.keys = ['session-key']

  // 如果中间件是generator则转换为promise以使用koa2
  const _use = app.use
  app.use = x => _use.call(app, convert(x))

  // 此处使用compose有问题 https://github.com/koajs/compose/issues/60
  // app.use(compose([
  //   cors({credentials: true}),
  //   logger(),
  //   bodyParser({
  //     onerror: function (err, ctx) {
  //       ctx.throw('body parse error', 422)
  //     }
  //   }),
  //   // 将静态文件服务app挂载在/
  //   mount('/', serve(path.resolve(__dirname, '..', 'public'), {index: 'index.html', gzip: true})),
  //   session(),
  //   passport.initialize(),
  //   passport.session(),
  // ]))
  app
    .use(cors({credentials: true}))
    .use(logger())
    .use(bodyParser({
      onerror: function (err, ctx) {
        ctx.throw('body parse error', 422)
      }
    }))
    // 将静态文件服务app挂载在/
    .use(mount('/', serve(env.app.public_dir, {index: 'index.html', gzip: true})))
    .use(session())
    .use(passport.initialize())
    .use(passport.session())


}