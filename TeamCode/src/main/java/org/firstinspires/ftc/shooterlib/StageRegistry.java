package org.firstinspires.ftc.shooterlib;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;

/**
 * The stages a spec string may name. Start from {@link #builtIns()} and register the team's own
 * stages on top; the spec parser then knows both.
 *
 * <pre>
 *   StageRegistry r = StageRegistry.builtIns().register("WindDrift", WindDrift::new);
 *   Pipeline p = r.parse("TagMedian; WindDrift(gustRpm=40); RecentTagGate(maxAgeMs=300)");
 * </pre>
 */
public final class StageRegistry {
    private final Map<String, Supplier<Stage>> factories = new LinkedHashMap<>();

    /** A registry holding only the library's built-in stages. */
    public static StageRegistry builtIns() { return BuiltInStages.register(new StageRegistry()); }

    /** Registers a stage under its {@link Stage#name()}. */
    public StageRegistry register(Supplier<Stage> factory) { return register(factory.get().name(), factory); }

    /** Registers a stage under an explicit name; the name must be an identifier and must be new. */
    public StageRegistry register(String name, Supplier<Stage> factory) {
        if (name == null || !name.matches("[A-Za-z][A-Za-z0-9]*"))
            throw new IllegalArgumentException("Stage name must be letters and digits, starting with a letter: '" + name + "'");
        if (factories.containsKey(name))
            throw new IllegalArgumentException("A stage named '" + name + "' is already registered");
        Stage probe = factory.get();
        if (!name.equals(probe.name()))
            throw new IllegalArgumentException("Registered as '" + name + "' but the stage says its name is '" + probe.name() + "'");
        if (!(probe instanceof RangeFilter) && !(probe instanceof VelocityModifier) && !(probe instanceof FireGate))
            throw new IllegalArgumentException(name + " implements none of RangeFilter, VelocityModifier or FireGate");
        factories.put(name, factory);
        return this;
    }

    public Set<String> names() { return Collections.unmodifiableSet(factories.keySet()); }
    public Map<String, Supplier<Stage>> asMap() { return Collections.unmodifiableMap(factories); }
    public boolean has(String name) { return factories.containsKey(name); }

    /** Builds a pipeline from a spec string using these stages. */
    public Pipeline parse(String spec) { return Pipeline.fromSpec(spec, factories); }
}
