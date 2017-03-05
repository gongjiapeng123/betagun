/**
 * Created by Administrator on 2016/4/19.
 * csv服务，用来导入和导出数据
 */


import { Injectable } from '@angular/core'
// import {Observable} from 'rxjs/Observable'
import { Subject } from 'rxjs/Subject'
import { BehaviorSubject } from 'rxjs/BehaviorSubject'
import * as io from 'socket.io-client'
import {
  JY901Data,
  ArdiunoData,
  OdomData,
} from './models'


@Injectable()
export class CsvService {

  /**
   * ros位姿 => three
   * @param rows 数据 [[p0, p1, p2], [p0, p1, p2]]
   */
  exportToCsv (rows) {
    const time = new Date()
    const filename = `${time.toLocaleDateString()} ${time.toLocaleTimeString()}.csv`
    const processRow = function (row) {
      let finalVal = ''
      for (let j = 0; j < row.length; j++) {
        let innerValue = row[j] === null ? '' : row[j].toString()
        if (row[j] instanceof Date) {
          innerValue = row[j].toLocaleString()
        }
        let result = innerValue.replace(/"/g, '""')
        if (result.search(/("|,|\n)/g) >= 0)
          result = '"' + result + '"'
        if (j > 0)
          finalVal += ','
        finalVal += result
      }
      return finalVal + '\n'
    }

    let csvFile = ''
    for (let i = 0; i < rows.length; i++) {
      csvFile += processRow(rows[i])
    }

    const blob = new Blob([csvFile], {type: 'text/csvcharset=utf-8'})
    if (navigator.msSaveBlob) {  // IE 10+
      navigator.msSaveBlob(blob, filename)
    } else {
      const link = document.createElement("a")
      if (link.download !== undefined) {
        // Browsers that support HTML5 download attribute
        const url = URL.createObjectURL(blob)
        link.setAttribute("href", url)
        link.setAttribute("download", filename)
        link.style.visibility = 'hidden'
        document.body.appendChild(link)
        link.click()
        document.body.removeChild(link)
      }
    }
  }
}