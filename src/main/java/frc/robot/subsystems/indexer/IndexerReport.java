package frc.robot.subsystems.indexer;

public class IndexerReport {

    public boolean isOperational = false;

    public void copyFrom(IndexerReport ref) {
        isOperational = ref.isOperational;
    }
}
