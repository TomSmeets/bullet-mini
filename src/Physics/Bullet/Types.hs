{-# LANGUAGE FlexibleContexts #-}
{-# LANGUAGE ForeignFunctionInterface #-}

module Physics.Bullet.Types where

import Foreign.C
import Foreign.Ptr

foreign import ccall "&free" freePtr :: FunPtr (Ptr CFloat -> IO ())

class ToCollisionObjectPointer a where
    toCollisionObjectPointer :: a -> Ptr ()
