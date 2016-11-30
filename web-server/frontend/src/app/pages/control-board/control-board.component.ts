import { Component, ViewEncapsulation } from '@angular/core'

@Component({
  selector: 'dashboard',
  encapsulation: ViewEncapsulation.None,
  styles: [require('./control-board.component.scss')],
  template: require('./control-board.component.html')
})
export class ControlBoard {

  constructor () {
  }

}
