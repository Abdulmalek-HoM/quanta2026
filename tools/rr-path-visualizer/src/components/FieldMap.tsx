import React, { useState } from 'react';
import type { Path, Waypoint, Pose } from '../types';

// Standard FTC Field is 144 inches (12ft)
const FIELD_SIZE_INCHES = 144;
const CANVAS_SIZE = 800;
const PIXELS_PER_INCH = CANVAS_SIZE / FIELD_SIZE_INCHES;

interface FieldMapProps {
    path: Path;
    setPath: React.Dispatch<React.SetStateAction<Path>>;
    selectedWaypointId: string | null;
    onSelectWaypoint: (id: string) => void;
}

const FieldMap: React.FC<FieldMapProps> = ({ path, setPath, selectedWaypointId, onSelectWaypoint }) => {
    const [mousePos, setMousePos] = useState<Pose>({ x: 0, y: 0, heading: 0 });

    // Coordinate Conversion Helpers
    // Screen: (0,0) Top-Left -> (800,800) Bottom-Right
    // Field: (-72, 72) Top-Left -> (72, -72) Bottom-Right (Standard Cartesian with Y up)

    const screenToRR = (screenX: number, screenY: number): Pose => {
        // RR X = (ScreenX - Center) / Scale, but X is typically 'forward'. 
        // Let's stick to: Map X axis = Screen X axis, Map Y axis = -Screen Y axis
        const centerX = CANVAS_SIZE / 2;
        const centerY = CANVAS_SIZE / 2;

        // x: right is positive
        const rrX = (screenX - centerX) / PIXELS_PER_INCH;
        // y: up is positive (so down on screen is negative)
        const rrY = (centerY - screenY) / PIXELS_PER_INCH;

        return { x: Number(rrX.toFixed(1)), y: Number(rrY.toFixed(1)), heading: 0 };
    };

    const rrToScreen = (pose: Pose) => {
        const centerX = CANVAS_SIZE / 2;
        const centerY = CANVAS_SIZE / 2;

        const screenX = centerX + (pose.x * PIXELS_PER_INCH);
        const screenY = centerY - (pose.y * PIXELS_PER_INCH);
        return { x: screenX, y: screenY };
    };

    const handleMouseMove = (e: React.MouseEvent) => {
        const rect = e.currentTarget.getBoundingClientRect();
        const x = e.clientX - rect.left;
        const y = e.clientY - rect.top;
        setMousePos(screenToRR(x, y));
    };

    const handleCanvasClick = (e: React.MouseEvent) => {
        const coords = screenToRR(e.nativeEvent.offsetX, e.nativeEvent.offsetY);

        const newWaypoint: Waypoint = {
            ...coords,
            id: crypto.randomUUID(),
            heading: 0,
            type: 'spline',
            actions: []
        };

        setPath(prev => {
            // If it's the very first point, set it as startPose too?
            if (prev.waypoints.length === 0 && prev.startPose.x === 0 && prev.startPose.y === 0) {
                return {
                    ...prev,
                    startPose: { ...coords, heading: 0 },
                    waypoints: [newWaypoint]
                };
            }
            return {
                ...prev,
                waypoints: [...prev.waypoints, newWaypoint]
            };
        });
    };

    return (
        <div
            className="field-container"
            style={{
                width: CANVAS_SIZE,
                height: CANVAS_SIZE,
                position: 'relative',
                backgroundColor: '#1e1e1e',
                border: '4px solid #444',
                borderRadius: '8px',
                boxShadow: '0 0 20px rgba(0,0,0,0.5)',
                cursor: 'crosshair',
                backgroundImage: `
          linear-gradient(rgba(255,255,255,0.05) 1px, transparent 1px),
          linear-gradient(90deg, rgba(255,255,255,0.05) 1px, transparent 1px)
        `,
                backgroundSize: `${PIXELS_PER_INCH * 24}px ${PIXELS_PER_INCH * 24}px`, // 2ft grid lines
                backgroundPosition: 'center center'
            }}
            onMouseMove={handleMouseMove}
            onClick={handleCanvasClick}
        >
            {/* Center Origin Marker */}
            <div style={{
                position: 'absolute',
                left: '50%', top: '50%',
                width: 10, height: 10,
                background: '#ff0055',
                transform: 'translate(-50%, -50%)',
                borderRadius: '50%',
                boxShadow: '0 0 10px #ff0055'
            }} />

            {/* SVG Overlay for Path */}
            <svg width={CANVAS_SIZE} height={CANVAS_SIZE} style={{ position: 'absolute', top: 0, left: 0, pointerEvents: 'none' }}>
                {/* Draw Path Segments */}
                {path.waypoints.map((wp, idx) => {
                    // If idx 0, connect from startPose
                    const prevPose = idx === 0 ? path.startPose : path.waypoints[idx - 1];

                    const p1 = rrToScreen(prevPose);
                    const p2 = rrToScreen(wp);

                    return (
                        <g key={wp.id}>
                            {/* Line Connection */}
                            <line
                                x1={p1.x} y1={p1.y}
                                x2={p2.x} y2={p2.y}
                                stroke="#4dabf7"
                                strokeWidth="3"
                                strokeOpacity="0.8"
                            />

                            {/* Waypoint Marker */}
                            <circle
                                cx={p2.x} cy={p2.y}
                                r={wp.id === selectedWaypointId ? "8" : "6"}
                                fill={wp.id === selectedWaypointId ? "#ffec99" : "#4dabf7"}
                                stroke="white"
                                strokeWidth="2"
                                style={{ cursor: 'pointer', pointerEvents: 'auto' }}
                                onClick={(e) => {
                                    e.stopPropagation(); // Prevent canvas click
                                    onSelectWaypoint(wp.id);
                                }}
                            />

                            {/* Heading Indicator (small arrow) */}
                            <line
                                x1={p2.x} y1={p2.y}
                                x2={p2.x + 15 * Math.cos(wp.heading * Math.PI / 180)}
                                y2={p2.y - 15 * Math.sin(wp.heading * Math.PI / 180)}
                                stroke="#ffec99"
                                strokeWidth="2"
                            />
                        </g>
                    );
                })}
            </svg>

            {/* Info HUD */}
            <div style={{
                position: 'absolute',
                top: 15,
                left: 15,
                background: 'rgba(0,0,0,0.8)',
                padding: '8px 12px',
                borderRadius: 6,
                color: '#e0e0e0',
                fontFamily: 'monospace',
                fontSize: '14px',
                border: '1px solid #333',
                pointerEvents: 'none'
            }}>
                <div style={{ color: '#888', fontSize: '0.8em', marginBottom: 2 }}>CURSOR</div>
                X: {mousePos.x.toFixed(1)}" <br />
                Y: {mousePos.y.toFixed(1)}"
            </div>
        </div>
    );
};

export default FieldMap;
