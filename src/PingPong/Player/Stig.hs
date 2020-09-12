module PingPong.Player.Stig (stig) where

import Data.Bool (bool)
import Data.Fixed (mod')

import Control.Lens
import Data.Geometry

import PingPong.Model
import PingPong.Player

import Graphics.Gloss (Color, makeColor)

-- Geometry Helpers

-- | Threshold function
threshold :: Float -> Float -> Float -> Float
threshold limit target val
    | abs diff < limit = target
    | otherwise = val
    where diff = target - val

-- | tau = 2 * pi
tau :: Float
tau = 2 * pi

-- | Normalize Angle in Radians between (-pi, pi]
-- Based on https://stackoverflow.com/a/2323034
normalizeAngle :: Float -> Float
normalizeAngle a = 
    let a1 = mod' a tau
        a2 = mod' (a + tau) tau
    in bool a2 (a2 - tau) (a2 > pi)

-- | Shortest Distance Between Two Angles in Radians [-pi, pi)
deltaAngle :: Float -> Float -> Float
deltaAngle a1 a2 
    | diff >= pi = diff - tau
    | diff < -pi = tau + diff
    | otherwise = diff
    where 
        a1N = normalizeAngle a1
        a2N = normalizeAngle a2
        diff = a1N - a2N

-- End of Geometry Helpers

-- Simulation Helpers

-- | Get the current Joint parameters as a Motion
getCurrentJoints :: Arm -> Motion
getCurrentJoints [] = []
getCurrentJoints (Joint _ rad : xs) = rad : getCurrentJoints xs
getCurrentJoints (Link _ _ : xs) = getCurrentJoints xs

-- | Motion Velocity Limit
motionLimit :: Float 
motionLimit = 1.0

-- | Apply Motion limits to avoid simulation problems
applyMotionLimits :: Motion -> Motion
applyMotionLimits = map f
    where f x = min motionLimit $ max (-motionLimit) x

-- End of Simulation Helpers

-- | Stig's player
stig :: Player
stig = Player stigArm stigFoot stigAction

paleBlue :: Color
paleBlue = makeColor 0.5 0.5 0.6 1

red :: Color
red = makeColor 1 0 0 1

hotPink :: Color
hotPink = makeColor 1.0 0.2 0.7 1

-- | Arm to use
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

-- | Separation from the center of the table
stigFoot :: Float
stigFoot = 1.3

-- | Common Threshold for stig functions
stigThreshold :: Float -> Float -> Float
stigThreshold = threshold 0.001

-- | Stig rest postion
stigRest :: Motion
stigRest = getCurrentJoints stigArm

-- | Get the a zeroed Motion list for Stig's arm
stigNoMotion :: Motion
stigNoMotion = map f stigRest
    where f = const 0

-- | Calculate Motion to Rest Position. !Warning: no limits are applied
armToStigRestMotion :: Arm -> Motion
armToStigRestMotion ar = zipWith f stigRest $ getCurrentJoints ar
    where 
        g = stigThreshold 0.0
        f = deltaAngle . g


stigAction :: BallState -> Arm -> IO Motion
stigAction bs arm = 
    return $ 
    let xdir = view xComponent $ dir bs
        toRest = armToStigRestMotion arm
        motion = bool [1, -1, 1, -1] toRest (xdir > 0)
    in applyMotionLimits motion -- Velocity limits