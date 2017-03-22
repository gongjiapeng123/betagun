# -*- coding: utf-8 -*-

import os

DIR = os.path.dirname(__file__)

LOGGING = {
    'version': 1,
    'disable_existing_loggers': True,
    'formatters': {
        'default': {
            'class': 'logging.Formatter',
            'format': '%(asctime)s %(name)s %(levelname)s: %(message)s'
        },
        'detailed': {
            'class': 'logging.Formatter',
            'format': '%(asctime)s %(name)-15s %(levelname)s pid:%(process)d tid:%(thread)d : %(message)s'
        },
        'simple': {
            'class': 'logging.Formatter',
            'format': '%(asctime)s %(levelname)-8s %(message)s'
        },
    },
    'handlers': {
        'console': {
            'class': 'logging.StreamHandler',
            'level': 'DEBUG',
            'formatter': 'default',
        },
        'file': {
            'class': 'logging.FileHandler',
            'filename': os.path.join(DIR, 'betagun.log'),
            'mode': 'w',
            'level': 'DEBUG',
            'formatter': 'default',
        },
        'rotate': {
            'class': 'logging.handlers.RotatingFileHandler',
            'filename': os.path.join(DIR, 'betagun_rotate.log'),
            'mode': 'a',
            'maxBytes': 1024 * 1024 * 8,
            'backupCount': 8,
            'encoding': None,
            'delay': False,
            'level': 'DEBUG',
            'formatter': 'default',
        },
        'mail': {
            'class': 'logging.handlers.SMTPHandler',
            'mailhost': 'smtp.163.com',
            'fromaddr': 'send@163.com',
            'toaddrs': ['recv@163.com', ],
            'subject': 'Online',
            'credentials': ('send@163.com', 'yourpin'),
            'secure': None,
            'level': 'DEBUG',
            'formatter': 'default',
        }
    },
    'loggers': {
        'betagun': {
            'propagate': False,
            'level': 'DEBUG',
            'handlers': ['console']
        },
        'betagun.mail': {
            'propagate': False,
            'level': 'DEBUG',
            'handlers': ['mail']
        }
    },
    'root': {
        'propagate': False,
        'level': 'DEBUG',
        'handlers': ['console']
    },
}