import Router from 'koa-router'

export default new Router()
  .get('/summary', async (ctx, next) => {

    let pv = Math.floor(Math.random() * 2000000) + 1500000
    let uv = Math.floor(pv / 3)
    let bounceRate = (Math.random() * 30 + 15) + ''

    let milliFormat = (input) => {
      return input && input.toString()
          .replace(/(^|\s)\d+/g, (m) => m.replace(/(?=(?!\b)(\d{3})+$)/g, ','))
    }
    ctx.body = {
      'status': 200,
      'data': {
        'totalVisitors': milliFormat(pv),
        'bounceRate': bounceRate.substr(0, 5) + '%',
        'uniqueVisitors': milliFormat(uv),
        'avgTime': '00:12:23'
      }
    }
  })
