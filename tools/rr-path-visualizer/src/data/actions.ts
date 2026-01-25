import type { Action } from '../types';

export const AVAILABLE_ACTIONS: Action[] = [
    {
        id: 'intake',
        name: 'Intake Artifact',
        codeSnippet: '// Intake Logic\n    .afterDisp(0, () -> {\n        intake.setPower(1.0);\n    })',
        description: 'Activates intake to grab an artifact.'
    },
    {
        id: 'shoot',
        name: 'Shoot',
        codeSnippet: '// Shoot Logic\n    .stopAndAdd(new ShootAction())',
        description: 'Fires the loaded artifact.'
    },
    {
        id: 'aim',
        name: 'Aim/Align',
        codeSnippet: '// Alignment Logic\n    .stopAndAdd(new AlignAction())',
        description: 'Aligns the robot to the target tag.'
    },
    {
        id: 'wait',
        name: 'Wait (1s)',
        codeSnippet: '.waitSeconds(1.0)',
        description: 'Waits for 1 second.'
    }
];
