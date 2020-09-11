module PingPong where

import PingPong.Model
import PingPong.Player
import PingPong.Player.MrStiff
import PingPong.Player.MsWavy
import PingPong.Player.Caller
import PingPong.Simulation

main :: IO ()
main = play msWavy msWavy