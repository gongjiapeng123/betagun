import { Routes, RouterModule }  from '@angular/router'

import { ControlBoard } from './control-board.component'

// noinspection TypeScriptValidateTypes
const routes: Routes = [
  {
    path: '',
    component: ControlBoard,
    children: [
      //{ path: 'treeview', component: TreeViewComponent }
    ]
  }
]

export const routing = RouterModule.forChild(routes)
