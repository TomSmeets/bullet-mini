{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.DynamicsWorld where

import qualified Language.C.Inline.Cpp as C

import Control.Monad.Trans
import Data.Monoid
import Data.Time
import Foreign.C
import Foreign.Ptr

{-
See

http://bulletphysics.org/Bullet/BulletFull/

https://github.com/bulletphysics/bullet3/blob/master/docs/Bullet_User_Manual.pdf

-}

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"
C.include "<BulletCollision/CollisionDispatch/btGhostObject.h>"

newtype DynamicsWorld = DynamicsWorld { unDynamicsWorld :: Ptr () } deriving Show

data DynamicsWorldConfig = DynamicsWorldConfig
  { dwGravity :: Float
  }
-- We're just implementing this for mempty;
-- the mappend instance isn't useful as it just takes the rightmost
-- DynamicsWorldConfig.
instance Monoid DynamicsWorldConfig where
  mempty = DynamicsWorldConfig
        { dwGravity = -9.8
        }
  mappend _ b = b

createDynamicsWorld :: (Functor m, MonadIO m) =>  DynamicsWorldConfig -> m DynamicsWorld
createDynamicsWorld DynamicsWorldConfig{..} = DynamicsWorld <$> liftIO [C.block| void* {
  btBroadphaseInterface* broadphase = new btDbvtBroadphase();

  btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
  btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

  btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

  btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(
    dispatcher, broadphase, solver, collisionConfiguration);

  dynamicsWorld->setGravity(btVector3(0,  $( float g ), 0));

  // This is required to use btGhostObjects
  dynamicsWorld->getPairCache()->setInternalGhostPairCallback(
    new btGhostPairCallback());

  return dynamicsWorld;
  } |]
  where
    g = realToFrac dwGravity

stepSimulationSimple :: MonadIO m => DynamicsWorld -> NominalDiffTime -> m ()
stepSimulationSimple (DynamicsWorld dynamicsWorld) (realToFrac -> dt) = liftIO [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void* dynamicsWorld);
    dynamicsWorld->stepSimulation($(float dt));
    }|]

stepSimulationWithTimestep :: MonadIO m => DynamicsWorld -> NominalDiffTime -> Int -> Float -> m ()
stepSimulationWithTimestep (DynamicsWorld dynamicsWorld) (realToFrac -> dt) (fromIntegral -> maxSubsteps) (realToFrac -> fixedTimeStep) = liftIO [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void* dynamicsWorld);
    dynamicsWorld->stepSimulation($(float dt), $(int maxSubsteps), $(float fixedTimeStep));
    }|]

-- I'm guessing I can get the solver/collisionConfig/dispatcher/broadphase from pointers in the dynamicsWorld
destroyDynamicsWorld :: DynamicsWorld -> IO ()
destroyDynamicsWorld (DynamicsWorld dynamicsWorld) = [C.block| void {
  btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void* dynamicsWorld);

  //delete solver;
  //delete collisionConfiguration;
  //delete dispatcher;
  //delete broadphase;

  delete dynamicsWorld;
} |]
