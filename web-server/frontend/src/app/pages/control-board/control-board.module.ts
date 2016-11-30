import { NgModule } from '@angular/core'
import { CommonModule } from '@angular/common'
import { FormsModule } from '@angular/forms'
import { NgaModule } from '../../theme/nga.module'

import { ControlBoard } from './control-board.component'
import { routing } from './control-board.routing'

import { BetaImageComponent } from './beta-image'
import { SteeringWheelComponent } from './steering-wheel'

@NgModule({
  imports: [
    CommonModule,
    FormsModule,
    NgaModule,
    routing
  ],
  declarations: [
    ControlBoard,
    BetaImageComponent,
    SteeringWheelComponent,
  ],
  providers: [
    
  ]
})
export default class ControlBoardModule {
}
