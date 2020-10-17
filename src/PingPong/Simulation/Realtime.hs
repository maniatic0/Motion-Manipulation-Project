module PingPong.Simulation.Realtime where

import PingPong.Model
import PingPong.Draw.Gloss
import PingPong.Simulation

import Graphics.Gloss (Display (InWindow), Picture, Color, white)
import Graphics.Gloss.Interface.IO.Simulate (simulateIO)
import Graphics.Gloss.Data.ViewPort

simulationRate :: Int
simulationRate = 50

windowDisplay :: Display
windowDisplay = InWindow "Window" (1600, 800) (100, 100)


play :: Player -> Player -> IO ()
play ip1 ip2 = do
  prepare ip1
  prepare ip2
  b <- startBall
  let initialState = defState {p1 = ip1, p2 = ip2, ball = b}
  simulateIO windowDisplay
             white
             simulationRate
             initialState
             (return . drawState)
             (const update)
    

