export interface Pose {
    x: number;
    y: number;
    heading: number; // Degrees
}

export interface Action {
    id: string;
    name: string;
    codeSnippet: string;
    description?: string;
}

export interface Waypoint extends Pose {
    id: string;
    type: 'spline' | 'line' | 'turn';
    actions?: Action[];
}

export interface Path {
    startPose: Pose;
    waypoints: Waypoint[];
}
