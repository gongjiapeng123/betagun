import { NgModule } from '@angular/core'
import { CommonModule } from '@angular/common'
import { FormsModule } from '@angular/forms'
import { NgaModule } from '../../theme/nga.module'
import { ControlBoard } from './control-board.component'
import { routing } from './control-board.routing'

import { BetaImageComponent } from './beta-image'
import { 
  SteeringDragComponent,
  SteeringWheelComponent,
  SteeringComponent
} from './steering'
import { TracePlotComponent } from './trace-plot'

import {
  SelectButtonModule,
  ButtonModule,
  InputSwitchModule,
} from 'primeng/primeng'

@NgModule({
  imports: [
    CommonModule,
    FormsModule,
    NgaModule,
    routing,

    SelectButtonModule,
    ButtonModule,
    InputSwitchModule,
  ],
  declarations: [
    ControlBoard,
    BetaImageComponent,
    SteeringComponent,
    SteeringWheelComponent,
    SteeringDragComponent,
    TracePlotComponent,
  ],
  providers: [

  ]
})
export class ControlBoardModule {
}
