package il.ac.bgu.cs.bp.leaderfollower.schema;

import java.util.concurrent.atomic.AtomicInteger;
import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.NamedQueries;
import javax.persistence.NamedQuery;

@Entity
@NamedQueries(value = {
    @NamedQuery(name = "MoveTowardsTarget", query = "SELECT t FROM Target t WHERE t.target != ''"),
    @NamedQuery(name = "MoveTowardsBall", query = "SELECT t FROM Target t WHERE t.target='ball'"),
    @NamedQuery(name = "MoveTowardsGoal", query = "SELECT t FROM Target t WHERE t.target='goal'"),
    @NamedQuery(name = "FacingGoal", query = "SELECT t FROM Target t WHERE t.target='goal' AND t.degreeFromPlayer < 10"),
    @NamedQuery(name = "BallIsFreeNearPlayer", query = "SELECT t FROM Target t WHERE t.target='ball' AND t.distanceFromPlayer < 4.3"),
    @NamedQuery(name = "UpdateTarget", query = "Update Target t SET t.target=:target, t.distanceFromPlayer=:distance, t.degreeFromPlayer=:degree"),
    @NamedQuery(name = "PurgeOldTargets", query = "Delete FROM Target"),
})
public class Target extends BasicEntity {
    private static final long serialVersionUID = -3194033486036115174L;
    private static AtomicInteger counter = new AtomicInteger(0);

    @Column
    public final String target;
    @Column
    public final Double distanceFromPlayer;
    @Column
    public final Double degreeFromPlayer;

    public Target(String target, Double distanceFromPlayer, Double degreeFromPlayer) {
        super(target + "_" + counter.incrementAndGet());
        this.target = target;
        this.degreeFromPlayer = degreeFromPlayer;
        this.distanceFromPlayer = distanceFromPlayer;
    }

    public Target() {
        this("", 0.0, 0.0);
    }

    @Override
    public String toString() {
        return target + ": distance=" + distanceFromPlayer + " , degree=" + degreeFromPlayer;
    }
}
