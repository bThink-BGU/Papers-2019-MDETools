package il.ac.bgu.cs.bp.leaderfollower.schema;

import javax.persistence.Column;
import javax.persistence.Entity;
import javax.persistence.NamedQueries;
import javax.persistence.NamedQuery;

@Entity
@NamedQueries(value = {
        @NamedQuery(name = "GoToTarget", query = "SELECT gt FROM Target gt WHERE gt.target!=''"),
        @NamedQuery(name = "GoToBall", query = "SELECT gt FROM Target gt WHERE gt.target='ball'"),
        @NamedQuery(name = "SetTarget", query = "Update Target gt set gt.target=:target"),})
public class Target extends BasicEntity {
    @Column
    public final String target;

    public Target() {
        super("Target");
        target = "";
    }

    @Override
    public String toString() {
        return "Target_" + target;
    }
}
