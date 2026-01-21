#!/usr/bin/env python3

# This file will interface drone movement to simple commands instead of raw position control through MAVSDK.
import asyncio
from mavsdk import System
from mavsdk.offboard import PositionNedYaw

class Controller:
    pass