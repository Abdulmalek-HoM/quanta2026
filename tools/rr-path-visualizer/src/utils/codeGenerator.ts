import type { Path, Waypoint } from '../types';

export const generateCode = (path: Path): string => {
    if (path.waypoints.length === 0) return "// No path drawn yet.";

    // const toRad = (deg: number) => (deg * Math.PI) / 180; // Unused for now if we use fmtRad directly
    const fmt = (n: number) => n.toFixed(2);
    const fmtRad = (n: number) => `Math.toRadians(${n.toFixed(1)})`;

    const start = path.startPose;

    let code = `// Valid for Roadrunner 1.0\n`;
    code += `TrajectoryActionBuilder tab = drive.actionBuilder(new Pose2d(${fmt(start.x)}, ${fmt(start.y)}, ${fmtRad(start.heading)}))\n`;

    path.waypoints.forEach((wp: Waypoint) => {
        // Basic SplineTo logic
        code += `    .splineTo(new Vector2d(${fmt(wp.x)}, ${fmt(wp.y)}), ${fmtRad(wp.heading)})\n`;

        // Append Actions
        if (wp.actions && wp.actions.length > 0) {
            wp.actions.forEach(act => {
                code += `    ${act.codeSnippet}\n`;
            });
        }
    });

    code += `    .build();`;

    return code;
};
