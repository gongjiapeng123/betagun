
import Router from 'koa-router'

export default new Router()
  .get('/', async (ctx, next) => {
    ctx.body = {
      "status": "home"
    }
  })
  .get('/app', async (ctx, next) => {
    ctx.body = {
      "status": "app"
    }
  })

