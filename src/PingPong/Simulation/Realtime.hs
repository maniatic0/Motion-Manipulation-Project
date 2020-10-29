module PingPong.Simulation.Realtime where

import PingPong.Model
import PingPong.Draw.Gloss
import PingPong.Simulation

import System.Exit
import Control.Monad

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
  initialState <- initBeforeGame $ defState {p1 = ip1, p2 = ip2}
  simulateIO windowDisplay
             white
             simulationRate
             initialState
             (return . drawState)
             realtimeUpdate

realtimeUpdate :: ViewPort -> Float -> State -> IO State
realtimeUpdate p f os = do
  ns <- update f os
  when (phase ns == GameOver) $ endGame ns
  return ns
    
endGame :: State -> IO ()
endGame st = do
  terminate $ p1 st
  terminate $ p2 st
  exitSuccess
