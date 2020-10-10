module PingPong where

import PingPong.Model

import PingPong.Simulation.Recording
--import PingPong.Simulation.Realtime

import PingPong.Player
import PingPong.Player.MrStiff
import PingPong.Player.MsWavy
import PingPong.Player.Caller
import PingPong.Player.ModelPlayer
import PingPong.Player.Stig

players :: [Player]
players = [mrStiff, msWavy, stig]

main :: IO ()
--main = play mrStiff msWavy
--main = play msWavy msWavy
main = play stig msWavy
