declare module "three-urdf-loader" {
  import { Loader, Object3D } from "three";

  export interface URDFJoint extends Object3D {
    setJointValue?: (value: number) => void;
  }

  export interface URDFRobot extends Object3D {
    joints: { [name: string]: URDFJoint };
    frames: { [name: string]: Object3D };
  }

  export default class URDFLoader extends Loader {
    constructor();
    load(
      url: string,
      onLoad: (result: URDFRobot) => void,
      onProgress?: (event: ProgressEvent<EventTarget>) => void,
      onError?: (event: unknown) => void
    ): void;
  }
}