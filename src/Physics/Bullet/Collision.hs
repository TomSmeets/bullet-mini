{-# LANGUAGE TemplateHaskell #-}
{-# LANGUAGE QuasiQuotes #-}
{-# LANGUAGE ForeignFunctionInterface #-}
{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE RecordWildCards #-}
{-# LANGUAGE DeriveGeneric #-}
{-# LANGUAGE GeneralizedNewtypeDeriving #-}
{-# LANGUAGE ViewPatterns #-}

module Physics.Bullet.Collision where

import qualified Language.C.Inline.Cpp as C

import Control.Monad.Trans
import Data.Binary
import Data.IORef
import Data.Monoid
import Foreign.C
import Foreign.Ptr
import GHC.Generics
import Linear.Extra
import Physics.Bullet.CollisionObject
import Physics.Bullet.DynamicsWorld
import Physics.Bullet.Types
import Text.RawString.QQ (r)

C.context (C.cppCtx <> C.funCtx)

C.include "<btBulletDynamicsCommon.h>"
C.include "<BulletCollision/CollisionDispatch/btGhostObject.h>"

-- I've disabled carrying the pointers in this structure so it can be serialized across the network.
-- We could also split it into 2 pieces, since it's probably often nice to query to the rigidbody directly.
data Collision = Collision
  -- { cbBodyA          :: !RigidBody
  -- , cbBodyB          :: !RigidBody
  { cbBodyAID        :: !CollisionObjectID
  , cbBodyBID        :: !CollisionObjectID
  , cbPositionOnA    :: !(V3 Float)
  , cbPositionOnB    :: !(V3 Float)
  , cbNormalOnB      :: !(V3 Float)
  , cbAppliedImpulse :: !Float
  } deriving (Show, Generic)
instance Binary Collision

-- | See computeOverlappingPairs. Adding these two so we can detect GhostObject intersections
-- even when the simulation is paused.
performDiscreteCollisionDetection :: MonadIO m => DynamicsWorld -> m ()
performDiscreteCollisionDetection (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void* dynamicsWorld);
    dynamicsWorld->performDiscreteCollisionDetection();
    }|]

-- | "the computeOverlappingPairs is usually already called by performDiscreteCollisionDetection
-- (or stepSimulation) it can be useful to use if you perform ray tests without collision detection/simulation"
computeOverlappingPairs :: MonadIO m => DynamicsWorld -> m ()
computeOverlappingPairs (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void* dynamicsWorld);
    dynamicsWorld->computeOverlappingPairs();
    }|]

updateAABBs :: MonadIO m => DynamicsWorld -> m ()
updateAABBs (DynamicsWorld dynamicsWorld) = liftIO [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void* dynamicsWorld);
    dynamicsWorld->updateAabbs();
    }|]

getCollisions :: MonadIO m => DynamicsWorld -> m [Collision]
getCollisions (DynamicsWorld dynamicsWorld) = liftIO $ do
    collisionsRef <- newIORef []
    -- let captureCollision objA objB objAID objBID aX aY aZ bX bY bZ nX nY nZ impulse = do
    let captureCollision objAID objBID aX aY aZ bX bY bZ nX nY nZ impulse = do
          let collision = Collision
                -- { cbBodyA = RigidBody objA
                -- , cbBodyB = RigidBody objB
                { cbBodyAID = CollisionObjectID (fromIntegral objAID)
                , cbBodyBID = CollisionObjectID (fromIntegral objBID)
                , cbPositionOnA    = realToFrac <$> V3 aX aY aZ
                , cbPositionOnB    = realToFrac <$> V3 bX bY bZ
                , cbNormalOnB      = realToFrac <$> V3 nX nY nZ
                , cbAppliedImpulse = realToFrac impulse
                }
          modifyIORef collisionsRef (collision:)

    [C.block| void {
        btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void* dynamicsWorld);

        int numCollisions = 0;

        int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
        for (int i = 0; i < numManifolds; i++) {

            btPersistentManifold* contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);
            int numContacts = contactManifold->getNumContacts();

            // We only return one contact for now. I believe there are up to 4.
            if (numContacts > 0) {
                btManifoldPoint& pt = contactManifold->getContactPoint(0);
                if (pt.getDistance()<0.f) {

                    const btCollisionObject* obA = contactManifold->getBody0();
                    const btCollisionObject* obB = contactManifold->getBody1();

                    const btVector3& ptA         = pt.getPositionWorldOnA();
                    const btVector3& ptB         = pt.getPositionWorldOnB();
                    const btVector3& nmB         = pt.m_normalWorldOnB;

                    btScalar impulse = pt.getAppliedImpulse();

                    //printf("Distance: %f Impulse: %f Normal: %f %f %f \n", pt.getDistance(), impulse,
                    //  nmB.getX(), nmB.getY(), nmB.getZ());

                    // fun:(void (*captureCollision)(void*, void*,
                    //                               int, int,
                    //                               float, float, float,
                    //                               float, float, float,
                    //                               float, float, float,
                    //                               float))(
                    //   (void*)obA,
                    //   (void*)obB,
                    $fun:(void (*captureCollision)(int, int,
                                                   float, float, float,
                                                   float, float, float,
                                                   float, float, float,
                                                   float)) (
                        obA->getUserIndex(),
                        obB->getUserIndex(),
                        ptA.getX(), ptA.getY(), ptA.getZ(),
                        ptB.getX(), ptB.getY(), ptB.getZ(),
                        nmB.getX(), nmB.getY(), nmB.getZ(),
                        impulse
                        );
                }
            }
        }
    }|]

    readIORef collisionsRef

data RayResult a = RayResult
  { rrCollisionObject :: CollisionObject
  , rrLocation        :: V3 a
  , rrNormal          :: V3 a
  }

rayTestClosest :: (RealFloat a, MonadIO m) => DynamicsWorld -> Ray a -> m (Maybe (RayResult a))
rayTestClosest (DynamicsWorld dynamicsWorld) ray = liftIO $ do
    ref <- newIORef Nothing
    let captureRayResult bodyPtr locX locY locZ norX norY norZ = if bodyPtr == nullPtr
          then return ()
          else writeIORef ref $ Just $ RayResult
                  { rrCollisionObject = CollisionObject bodyPtr
                  , rrLocation        = realToFrac <$> V3 locX locY locZ
                  , rrNormal          = realToFrac <$> V3 norX norY norZ
                  }
    [C.block| void {
        btCollisionWorld* world = (btCollisionWorld*)$(void* dynamicsWorld);
        btVector3 from = btVector3($(float fx), $(float fy), $(float fz));
        btVector3 to   = btVector3($(float tx), $(float ty), $(float tz));

        btCollisionWorld::ClosestRayResultCallback callback(from, to);
        world->rayTest(from, to, callback);

        btVector3 point  = callback.m_hitPointWorld;
        btVector3 normal = callback.m_hitNormalWorld;
        $fun:(void (*captureRayResult)(void*, float, float, float,
                                              float, float, float)) (
                (void*)callback.m_collisionObject,
                point.getX(),  point.getY(),  point.getZ(),
                normal.getX(), normal.getY(), normal.getZ()
            );
    }|]
    readIORef ref
    where (V3 fx fy fz) = realToFrac <$> rayOrigin ray
          (V3 tx ty tz) = realToFrac <$> projectRay ray 1000

C.verbatim [r|

typedef void (*CaptureCollisionPtr)(int, int,
                                    float, float, float,
                                    float, float, float,
                                    float, float, float,
                                    float);

struct GatherContactResultsCallback : public btCollisionWorld::ContactResultCallback {

    GatherContactResultsCallback(btCollisionObject* tgtBody , CaptureCollisionPtr context)
        : btCollisionWorld::ContactResultCallback(), body(tgtBody), funPtr(context) { }

    btCollisionObject* body; // The body the sensor is monitoring
    CaptureCollisionPtr funPtr; // External information for contact processing

    //virtual bool needsCollision(btBroadphaseProxy* proxy) const {
    //}

    virtual btScalar addSingleResult(btManifoldPoint& pt,
        const btCollisionObjectWrapper* colObj0,int partId0,int index0,
        const btCollisionObjectWrapper* colObj1,int partId1,int index1)
    {
        const btCollisionObject* obA = colObj0->m_collisionObject;
        const btCollisionObject* obB = colObj1->m_collisionObject;

        const btVector3& ptA         = pt.getPositionWorldOnA();
        const btVector3& ptB         = pt.getPositionWorldOnB();
        const btVector3& nmB         = pt.m_normalWorldOnB;

        btScalar impulse = pt.getAppliedImpulse();

        //printf("Distance: %f Impulse: %f Normal: %f %f %f \n", pt.getDistance(), impulse,
        //  nmB.getX(), nmB.getY(), nmB.getZ());

        funPtr(
            obA->getUserIndex(),
            obB->getUserIndex(),
            ptA.getX(), ptA.getY(), ptA.getZ(),
            ptB.getX(), ptB.getY(), ptB.getZ(),
            nmB.getX(), nmB.getY(), nmB.getZ(),
            impulse
            );

        // do stuff with the collision point


        return 0; // There was a planned purpose for the return value of addSingleResult, but it is not used so you can ignore it.
    }
};

|]

contactTest :: (MonadIO m, ToCollisionObjectPointer a) => DynamicsWorld -> a -> m [Collision]
contactTest (DynamicsWorld dynamicsWorld) (toCollisionObjectPointer -> collisionObject) = liftIO $ do
  collisionsRef <- newIORef []
  let captureCollision objAID objBID aX aY aZ bX bY bZ nX nY nZ impulse = do
        let collision = Collision
              -- { cbBodyA = RigidBody objA
              -- , cbBodyB = RigidBody objB
              { cbBodyAID = CollisionObjectID (fromIntegral objAID)
              , cbBodyBID = CollisionObjectID (fromIntegral objBID)
              , cbPositionOnA    = realToFrac <$> V3 aX aY aZ
              , cbPositionOnB    = realToFrac <$> V3 bX bY bZ
              , cbNormalOnB      = realToFrac <$> V3 nX nY nZ
              , cbAppliedImpulse = realToFrac impulse
              }
        modifyIORef collisionsRef (collision:)

  [C.block| void {
    btDiscreteDynamicsWorld* dynamicsWorld = (btDiscreteDynamicsWorld*)$(void* dynamicsWorld);
    btCollisionObject* collisionObject = (btCollisionObject*)$(void* collisionObject);

    CaptureCollisionPtr captureCollisionPtr = $fun:(void (*captureCollision)(int, int,
                                       float, float, float,
                                       float, float, float,
                                       float, float, float,
                                       float));
    GatherContactResultsCallback callback(collisionObject, captureCollisionPtr);
    dynamicsWorld->contactTest(collisionObject, callback);

  }|]
  readIORef collisionsRef
