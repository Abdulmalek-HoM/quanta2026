import { useState } from 'react'
import './index.css'
import FieldMap from './components/FieldMap'
import ActionSidebar from './components/ActionSidebar'
import type { Path, Waypoint } from './types'
import { generateCode } from './utils/codeGenerator'

function App() {
  const [path, setPath] = useState<Path>({
    startPose: { x: 0, y: 0, heading: 0 },
    waypoints: []
  });
  const [selectedWaypointId, setSelectedWaypointId] = useState<string | null>(null);

  const handleUpdateWaypoint = (id: string, updates: Partial<Waypoint>) => {
    setPath(prev => ({
      ...prev,
      waypoints: prev.waypoints.map(wp => wp.id === id ? { ...wp, ...updates } : wp)
    }));
  };

  return (
    <div style={{ display: 'flex', width: '100%', height: '100vh', overflow: 'hidden' }}>

      {/* Main Field Area */}
      <div style={{ flex: 1, display: 'flex', justifyContent: 'center', alignItems: 'center', backgroundColor: '#111', position: 'relative' }}>
        <FieldMap
          path={path}
          setPath={setPath}
          selectedWaypointId={selectedWaypointId}
          onSelectWaypoint={setSelectedWaypointId}
        />
      </div>

      {/* Right Panel: Split between Code and Actions */}
      <div style={{
        width: '400px',
        backgroundColor: '#1e1e1e',
        borderLeft: '1px solid #333',
        display: 'flex',
        flexDirection: 'column',
        boxShadow: '-5px 0 15px rgba(0,0,0,0.3)',
        zIndex: 10
      }}>
        {/* Action Configuration (Top Half) */}
        <div style={{ flex: 1, borderBottom: '1px solid #333', overflow: 'hidden' }}>
          <ActionSidebar
            selectedWaypointId={selectedWaypointId}
            waypoints={path.waypoints}
            onUpdateWaypoint={handleUpdateWaypoint}
          />
        </div>

        {/* Generated Code (Bottom Half) */}
        <div style={{ height: '40%', display: 'flex', flexDirection: 'column', padding: '20px', backgroundColor: '#181818' }}>
          <h2 style={{ marginTop: 0, color: '#fff', fontSize: '1.2rem' }}>Generated Code</h2>
          <div style={{
            flex: 1,
            backgroundColor: '#111',
            borderRadius: '6px',
            padding: '15px',
            fontFamily: 'monospace',
            fontSize: '0.85rem',
            color: '#a5d6ff',
            overflow: 'auto',
            whiteSpace: 'pre-wrap',
            border: '1px solid #333'
          }}>
            {generateCode(path)}
          </div>

          <div style={{ marginTop: '10px', display: 'flex', gap: '10px' }}>
            <button onClick={() => navigator.clipboard.writeText(generateCode(path))}>
              Copy
            </button>
            <button
              style={{ backgroundColor: '#e03131', color: 'white' }}
              onClick={() => {
                setPath({ startPose: { x: 0, y: 0, heading: 0 }, waypoints: [] });
                setSelectedWaypointId(null);
              }}
            >
              Clear
            </button>
          </div>
        </div>
      </div>

    </div>
  )
}

export default App
