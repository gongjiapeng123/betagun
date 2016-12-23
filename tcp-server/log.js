const path = require('path')
const log4js = require('log4js')

const LOG_DIR = '/root/logs'

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
      filename: path.join(LOG_DIR, 'betagun_tcp.log'),
      maxLogSize: 10 * 1024 * 1024,  // = 10Mb
      numBackups: 5,  // keep five backup files
      compress: true,  // compress the backups
      encoding: 'utf-8',
      mode: parseInt('0640', 8),
      flags: 'w+',
    },
  ],
  levels: {
    main: 'ALL',
  }
})

module.exports = log4js.getLogger('main')


