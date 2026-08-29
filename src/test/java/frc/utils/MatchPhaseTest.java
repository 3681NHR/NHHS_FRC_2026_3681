package frc.utils;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

public class MatchPhaseTest {

    @Test
    public void phaseTimes_areConsistent() {
        for (MatchPhase p : MatchPhase.values()) {
            if (p != MatchPhase.UNKNOWN) {
                assertTrue(p.getStartTime() < p.getEndTime(), p.name() + " start < end");
            }
            // UNKNOWN is terminator with 0/0; ENDGAME->UNKNOWN intentionally breaks chain
            if (p.getNext() != null && p.getNext() != MatchPhase.UNKNOWN && p != MatchPhase.ENDGAME) {
                assertEquals(p.getEndTime(), p.getNext().getStartTime(), p.name() + " end == next start");
            }
        }
    }

    @Test
    public void autoTransitionsToTeleop() {
        assertEquals(MatchPhase.AUTO_TELE_TRANSITION, MatchPhase.AUTO.getNext());
        assertEquals(MatchPhase.TRANSITION_SHIFT, MatchPhase.AUTO_TELE_TRANSITION.getNext());
        assertEquals(MatchPhase.SHIFT1, MatchPhase.TRANSITION_SHIFT.getNext());
    }

    @Test
    public void chainEndsAtUnknown() {
        MatchPhase cur = MatchPhase.AUTO;
        int steps = 0;
        while (cur != null && cur != MatchPhase.UNKNOWN && steps < 20) {
            cur = cur.getNext();
            steps++;
        }
        assertEquals(MatchPhase.UNKNOWN, cur);
    }

    @Test
    public void endgameIsLastBeforeUnknown() {
        assertEquals(MatchPhase.UNKNOWN, MatchPhase.ENDGAME.getNext());
    }
}
