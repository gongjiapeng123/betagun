'use strict'

import Router from 'koa-router'
import passport from 'koa-passport'

export default new Router()
  .post('/login', async (ctx, next) => {
    let middleware = passport.authenticate('local', async (user, info) => {
      if (user === false) {
        ctx.body = {
          'status': 400
        }
      } else {
        await ctx.login(user)  // passport为登录用户初始化session
        ctx.body = {
          user: user
        }
      }
    })
    await middleware.call(this, ctx, next)
  })
  .get('/logout', async (ctx, newt) => {
    ctx.logout()
    ctx.redirect('/')
  })
  .get('/status', async (ctx, next) => {
    ctx.body = {
      "isLogin": ctx.isAuthenticated()
    }
  })

