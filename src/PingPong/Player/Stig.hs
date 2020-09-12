module PingPong.Player.Stig (stig) where

import Data.Bool (bool)

import Control.Lens
import Data.Geometry

import PingPong.Model
import PingPong.Player

import Graphics.Gloss (Color, makeColor)

stig :: Player
stig = Player stigArm stigFoot noAction

paleBlue :: Color
paleBlue = makeColor 0.5 0.5 0.6 1

red :: Color
red = makeColor 1 0 0 1

hotPink :: Color
hotPink = makeColor 1.0 0.2 0.7 1

stigArm :: Arm
stigArm = [ Joint red (-0.3) -- (0.1)
           , Link paleBlue 0.5
           , Joint red 1.3 -- (0.1)
           , Link paleBlue 0.4
           , Joint red 0.9 -- (-0.1)
           , Link paleBlue 0.2
           , Joint red 0.5 -- (-0.1)
           , Link hotPink 0.1 -- Bat
           ]

stigFoot :: Float
stigFoot = 1.3

-- | Get the current Joint parameters as a Motion
getCurrentJoints :: Arm -> Motion
getCurrentJoints [] = []
getCurrentJoints (Joint _ rad : xs) = rad : getCurrentJoints xs
getCurrentJoints (Link _ _ : xs) = getCurrentJoints xs

-- | Get the an empty Motion list from Stig's arm
stigNoMotion :: Motion
stigNoMotion = map f $ getCurrentJoints stigArm
    where f = const 0

-- | Motion Velocity Limit
motionLimit :: Float 
motionLimit = 1.0

-- | Apply Motion limits to avoid simulation problems
applyMotionLimits :: Motion -> Motion
applyMotionLimits = map f
    where f x = min motionLimit $ max (-motionLimit) x

noAction :: BallState -> Arm -> IO Motion
noAction bs arm = 
    return $ 
    let xdir = view xComponent $ dir bs
        curJoints = getCurrentJoints arm
    in bool stigNoMotion [1, 1, 1, -1] (xdir > 0)