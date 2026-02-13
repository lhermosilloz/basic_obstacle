#!/usr/bin/env python3
import asyncio
from mavsdk import System

async def run():
    drone = System()
    
    print("Attempting connection to 0.0.0.0:14550...")
    await drone.connect(system_address="udpin://0.0.0.0:14550")
    
    print("Waiting for connection (30s timeout)...")
    start_time = asyncio.get_event_loop().time()
    
    async for state in drone.core.connection_state():
        elapsed = asyncio.get_event_loop().time() - start_time
        print(f"[{elapsed:.1f}s] Connection state: is_connected={state.is_connected}")
        
        if state.is_connected:
            print("SUCCESS: Drone connected!")
            break
            
        if elapsed > 30:
            print("TIMEOUT: No connection after 30 seconds")
            break

if __name__ == "__main__":
    asyncio.run(run())