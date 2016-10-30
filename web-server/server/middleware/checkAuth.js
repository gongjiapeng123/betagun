

export default function checkAuth () {
  return async function (ctx, next) {

    if (ctx.isAuthenticated()) {
      await next()
    } else {
      ctx.body = {
        status: 401
      }
    }
  }
}