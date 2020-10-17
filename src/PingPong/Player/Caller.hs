module PingPong.Player.Caller (caller) where

import System.Process

import Data.Geometry

import PingPong.Model
import PingPong.Player

import Graphics.Gloss (Color, makeColor)

caller :: Player
caller = defaultPlayer
  { arm = callArm
  , foot = callFoot
  , action = callAction
  }

externalCommand :: String
externalCommand = "echo \"hello world\""


grey :: Color
grey = makeColor 0.5 0.5 0.5 1

callArm :: Arm
callArm = [ Link grey 0.3
          , Joint grey 0
          , Link grey 0.3
          , Joint grey 0
          , Link grey 0.3
          , Joint grey 0
          , Link grey 0.1
          ]

callFoot :: Float
callFoot = 1.4



callAction :: Float -> (Float, Item) -> BallState -> Arm -> IO Motion
callAction time hit bs arm = do
  let state = writeState time hit bs arm
  writeFile "in" state
  callCommand externalCommand
  out <- readFile "out"
  let motion = readMotion out
  return motion
