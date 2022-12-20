import { DispenserState } from "./DispenserState";
import { ErrorState } from "./ErrorState";
import { RobotState } from "./RobotState";

export default function StatePage(props: any) {
    return <div>
        <RobotState {...props}/>
        <DispenserState />
        <ErrorState />
    </div>
}