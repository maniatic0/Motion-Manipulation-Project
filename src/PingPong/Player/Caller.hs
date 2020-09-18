module PingPong.Player.Caller (caller) where

import System.Process

import Data.Geometry

import PingPong.Model
import PingPong.Player

import Graphics.Gloss (Color, makeColor)

caller :: Player
caller = Player callArm callFoot callAction noCollide

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



callAction :: BallState -> Arm -> IO Motion
callAction bs arm = do
  let state = writeState bs arm
  writeFile "in" state
  callCommand externalCommand
  out <- readFile "out"
  let motion = readMotion out
  return motion


noCollide :: (Float, Point 2 Float, LineSegment 2 () Float) 
          -> (Float, Point 2 Float, LineSegment 2 () Float) 
          -> Point 2 Float
noCollide (t1, p1, s1) (t2, p2, s2) = p2