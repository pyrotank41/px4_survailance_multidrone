import asyncio
from typing import KeysView
import pygame
import time
import mavsdk
from mavsdk import System

drone = System()

pygame.init()
window = pygame.display.set_mode((300, 300))

print("yoyo")

async def setup():
    """
    General configurations, setups, and connections are done here.
    :return:
    """

    await drone.connect(system_address="udp://:14030")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

pitch = 0
yaw = 0

async def yaw_pitch_check(yaw, pitch):
    if yaw >= 360 or yaw <= -360:
        yaw = 0

    if pitch <= -90:
        pitch = -89.9
    if pitch >= 45:
        pitch =45

    return yaw, pitch


async def main():
    """
    Launching a specific pygame window retrieves and works according to the received commands.
    :return:
    """
    global pitch
    global yaw
    
    running = True
    await drone.gimbal.take_control( mavsdk.gimbal.ControlMode(1))

    while running:
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()

        # While being in air and landing mode the drone is not likely to takeoff again, so
        # a condition check is required here to avoid such a condition.

        if keys[pygame.K_UP] and (await print_in_air(drone) != True):
            await takeoff()

        elif keys[pygame.K_DOWN]:
            await land()

        elif keys[pygame.K_RIGHT]:
            await print_in_air(drone)

        elif keys[pygame.K_i]:
            await info(drone)
        
        elif keys[pygame.K_d]:
            
            # yaw  += 0.1
            yaw, pitch = await yaw_pitch_check(yaw + 0.1, pitch)
            await drone.gimbal.set_pitch_and_yaw(pitch, yaw)
            print(f"yaw = {yaw}")
            time.sleep(0.001)

        elif keys[pygame.K_a]:
            # yaw -= 0.1
            yaw, pitch = await yaw_pitch_check(yaw - 0.1, pitch)
            await drone.gimbal.set_pitch_and_yaw(pitch, yaw)
            print(f"yaw = {yaw}")
            time.sleep(0.001)

        elif keys[pygame.K_w]:
            # pitch  += 0.1
            yaw, pitch = await yaw_pitch_check(yaw, pitch + 0.1)
            await drone.gimbal.set_pitch_and_yaw(pitch, yaw)
            print(f"pitch = {pitch}")
            time.sleep(0.001)

        elif keys[pygame.K_s]:
            #pitch -= 0.1
            yaw, pitch = await yaw_pitch_check(yaw, pitch - 0.1)
            await drone.gimbal.set_pitch_and_yaw(pitch, yaw)
            print(f"pitch = {pitch}")
            time.sleep(0.001)

async def takeoff():
    """
    Default takeoff command seperated and taken from takeoff_and_land.py
    :return:
    """

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()




async def land():
    """
    Default land command seperated and taken from takeoff_and_land.py
    :return:
    """

    await drone.action.land()


async def print_in_air(drone=drone):
    async for in_air in drone.telemetry.in_air():
        print(f"In air: {in_air}")
        return in_air


async def info(drone=drone):
    """
    This is the combination of the print_battery, print_in_air, print_gps_info, and print_position functions aimed
    to display all of the counted data/information at the same exact time.
    :param drone:
    :return:
    """
    
    
    await print_battery(drone)
    await print_in_air(drone)
    await print_gps_info(drone)
    await print_position(drone)

    return True


async def print_battery(drone=drone):
    """
    Default print_battery command seperated and taken from telemetry.py
    :param drone:
    :return:
    """

    async for battery in drone.telemetry.battery():
        print(f"Battery: {battery.remaining_percent}")
        return battery.remaining_percent


async def print_gps_info(drone=drone):
    """
    Default print_gps_info command seperated and taken from telemetry.py
    :param drone:
    :return:
    """

    async for gps_info in drone.telemetry.gps_info():
        print(f"GPS info: {gps_info}")
        return gps_info


async def print_position(drone=drone):
    """
    Default print_position command seperated and taken from telemetry.py
    :param drone:
    :return:
    """

    async for position in drone.telemetry.position():
        print(position)
        return position


if __name__ == "__main__":

    loop = asyncio.get_event_loop()
    print("looping setup()")
    loop.run_until_complete(setup())
    loop.run_until_complete(main())
    loop.run_until_complete(print_in_air(drone))
    loop.run_until_complete(info(drone))
    loop.run_until_complete(print_battery())
    loop.run_until_complete(print_gps_info())
    loop.run_until_complete(print_position())
