import sys
sys.path.append('C:\SOFA\plugins\SofaPython3\lib\python3\site-packages')

# pylint: disable=no-member

from time import perf_counter
import pygame
import numpy as np
import eve
import eve.visualisation

vessel_tree = eve.vesseltree.AorticArch()
device = eve.intervention.device.JWire(beams_per_mm_straight=0.5)
device2 = eve.intervention.device.JWire(
    name="cath",
    visu_edges_per_mm=0.5,
    tip_outer_diameter=1.2,
    straight_outer_diameter=1.2,
    tip_inner_diameter=1.0,
    straight_inner_diameter=1.0,
    color=(1.0, 0.0, 0.0),
)

intervention = eve.intervention.Intervention(
    vessel_tree=vessel_tree, devices=[device, device2], lao_rao_deg=-5, cra_cau_deg=20
)
visualisation = eve.visualisation.SofaPygame(intervention)

vessel_tree.reset(0, None)
intervention.reset(0)
visualisation.reset(0)

while True:
    trans = 0.0
    rot = 0.0
    pygame.event.get()
    keys_pressed = pygame.key.get_pressed()

    if keys_pressed[pygame.K_ESCAPE]:
        break
    if keys_pressed[pygame.K_UP]:
        trans += 25
    if keys_pressed[pygame.K_DOWN]:
        trans -= 25
    if keys_pressed[pygame.K_LEFT]:
        rot += 1 * 3.14
    if keys_pressed[pygame.K_RIGHT]:
        rot -= 1 * 3.14

    if keys_pressed[pygame.K_v]:
        action = ((0, 0), (trans, rot))
    else:
        action = ((trans, rot), (0, 0))

    print(action)
    vessel_tree.step()
    intervention.step(action)    
    visualisation.render()

intervention.close()
visualisation.close()
