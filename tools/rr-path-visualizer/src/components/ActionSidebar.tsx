import React from 'react';
import type { Action, Waypoint } from '../types';
import { AVAILABLE_ACTIONS } from '../data/actions';

interface ActionSidebarProps {
    selectedWaypointId: string | null;
    waypoints: Waypoint[];
    onUpdateWaypoint: (id: string, updates: Partial<Waypoint>) => void;
}

const ActionSidebar: React.FC<ActionSidebarProps> = ({ selectedWaypointId, waypoints, onUpdateWaypoint }) => {
    const selectedWaypoint = waypoints.find(wp => wp.id === selectedWaypointId);

    if (!selectedWaypoint) {
        return (
            <div style={{ padding: '20px', color: '#888', textAlign: 'center' }}>
                <p>Select a waypoint on the map to configure actions.</p>
            </div>
        );
    }

    const handleAddAction = (action: Action) => {
        const currentActions = selectedWaypoint.actions || [];
        // clone to avoid ref issues
        const newAction = { ...action, id: crypto.randomUUID() }; // Unique instance ID

        // For simplicity, we just store the Action Reference or a full object. 
        // Let's store full object so we can modify params later if needed.
        onUpdateWaypoint(selectedWaypoint.id, {
            actions: [...currentActions, newAction]
        });
    };

    const handleRemoveAction = (actionIndex: number) => {
        const currentActions = selectedWaypoint.actions || [];
        const newActions = [...currentActions];
        newActions.splice(actionIndex, 1);
        onUpdateWaypoint(selectedWaypoint.id, { actions: newActions });
    };

    return (
        <div style={{ display: 'flex', flexDirection: 'column', height: '100%', color: '#e0e0e0' }}>
            <div style={{ padding: '15px', borderBottom: '1px solid #444' }}>
                <h3 style={{ margin: 0 }}>Waypoint Configuration</h3>
                <p style={{ margin: '5px 0 0', fontSize: '0.8em', color: '#aaa' }}>
                    ID: {selectedWaypoint.id.slice(0, 8)}
                </p>
            </div>

            <div style={{ padding: '15px', flex: 1, overflowY: 'auto' }}>
                <h4 style={{ marginTop: 0 }}>Assigned Actions</h4>
                <div style={{ display: 'flex', flexDirection: 'column', gap: '8px', marginBottom: '20px' }}>
                    {(selectedWaypoint.actions && selectedWaypoint.actions.length > 0) ? (
                        selectedWaypoint.actions.map((act, idx) => (
                            <div key={idx} style={{
                                background: '#333',
                                padding: '8px',
                                borderRadius: '4px',
                                display: 'flex',
                                justifyContent: 'space-between',
                                alignItems: 'center'
                            }}>
                                <span style={{ fontSize: '0.9em' }}>{act.name}</span>
                                <button
                                    onClick={() => handleRemoveAction(idx)}
                                    style={{ padding: '2px 6px', fontSize: '0.8em', background: '#e03131', color: 'white', border: 'none' }}
                                >
                                    X
                                </button>
                            </div>
                        ))
                    ) : (
                        <div style={{ fontSize: '0.85em', color: '#666', fontStyle: 'italic' }}>No actions assigned</div>
                    )}
                </div>

                <h4 style={{ marginBottom: '10px' }}>Add Action</h4>
                <div style={{ display: 'grid', gridTemplateColumns: '1fr', gap: '8px' }}>
                    {AVAILABLE_ACTIONS.map(action => (
                        <button
                            key={action.id}
                            onClick={() => handleAddAction(action)}
                            style={{
                                textAlign: 'left',
                                background: '#2a2a2a',
                                border: '1px solid #444',
                                padding: '10px',
                                display: 'flex',
                                justifyContent: 'space-between',
                                alignItems: 'center'
                            }}
                        >
                            <div>
                                <div style={{ fontWeight: 'bold', fontSize: '0.9em' }}>{action.name}</div>
                                <div style={{ fontSize: '0.75em', color: '#888' }}>{action.description}</div>
                            </div>
                            <span style={{ fontSize: '1.2em', color: '#646cff' }}>+</span>
                        </button>
                    ))}
                </div>
            </div>
        </div>
    );
};

export default ActionSidebar;
