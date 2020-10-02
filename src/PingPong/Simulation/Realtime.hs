module PingPong.Simulation.Realtime where

import PingPong.Model
import PingPong.Draw.Gloss
import PingPong.Simulation

import Graphics.Gloss (Display (InWindow), Picture, Color, white)
import Graphics.Gloss.Interface.IO.Simulate (simulateIO)
import Graphics.Gloss.Data.ViewPort

play :: Player -> Player -> IO ()
play ip1 ip2 = simulateIO
  windowDisplay
  white
  simulationRate
  initialState
  (return . center . drawState)
  (const update)
  where
    simulationRate :: Int
    simulationRate = 50

    initialState :: State
    initialState = State ip1 ip2 startBall

windowDisplay :: Display
windowDisplay = InWindow "Window" (1600, 800) (100, 100)

