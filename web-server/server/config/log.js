import path from 'path'
import log4js from 'log4js'
import env from './env'

export default function configLog () {

  log4js.configure({
    appenders: [
      {
        type: 'console',
        category: 'console',
      },
      {
        type: 'console',
        category: 'default',
      },
      {
        category: 'default',
        type: 'file',
        filename: path.join(env.LOG_DIR, 'default.log'),
        maxLogSize: 104857500,
        backups: 100
      },
      // {
      //   category: 'log_stat',
      //   type: 'datefile',
      //   filename: path.join(LOG_DIR, 'log_stat/stat.log'),
      // },
      // {
      //   category: 'log_trace',
      //   type: 'datefile',
      //   filename: path.join(LOG_DIR, 'log_trace/trace.log'),
      // },
      // {
      //   category: 'log_error',
      //   type: 'datefile',
      //   filename: path.join(LOG_DIR, 'log_error/error.log'),
      // },
      // {
      //   category: 'log_todo',
      //   type: 'datefile',
      //   filename: path.join(LOG_DIR, 'log_todo/todo.log'),
      // }
    ],
    replaceConsole: true,
    levels: {
      default: 'ALL',
      log_info: 'ALL',
      log_stat: 'ALL',
      log_trace: 'ALL',
      log_error: 'ALL',
      log_todo: 'ALL',
    }
  })

  return log4js.getLogger('default')
}


