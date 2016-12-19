import * as path from 'path'
import * as log4js from 'log4js'
import { env } from './env'

log4js.configure({
  appenders: [
    {
      type: 'stdout',
      encoding: 'utf-8',
      category: 'main',
    },
    {
      category: 'main',
      type: 'file',
      filename: path.join(env.LOG_DIR, 'main.log'),
      maxLogSize: 10 * 1024 * 1024,  // = 10Mb
      numBackups: 5,  // keep five backup files
      compress: true,  // compress the backups
      encoding: 'utf-8',
      mode: parseInt('0640', 8),
      flags: 'w+',
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
  levels: {
    main: 'ALL',
    log_info: 'ALL',
    log_stat: 'ALL',
    log_trace: 'ALL',
    log_error: 'ALL',
    log_todo: 'ALL',
  }
})

export const logger = log4js.getLogger('main')


