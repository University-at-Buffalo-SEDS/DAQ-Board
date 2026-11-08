"""Exact current cohort source dependencies for existing isolated SD owner tests.
No thermal seal/activation is invented by this fixture; each test retains its
original row, clock, queue, retry and admission controls.
"""
from router_lifecycle_fixture import function

def pressure_cohort(source):
    begin=source.index('static struct { uint32_t current, old, cutoff, flags;')
    end=source.index('static void sd_route_epoch_advance(',begin)
    definitions=source[begin:end]
    return definitions+'\n'+'\n'.join(function(source,n) for n in (
        'sd_capture_old_locked','sd_capture_retain_locked','sd_card_capture_release','sd_card_capture_fault'))+'\n'


def raw_enqueue(source):
    # Existing raw-queue cases always supply their immutable calibration. This
    # collaborator only supplies the unused current snapshot when no seal exists;
    # actual capture/retirement/cutover behavior is exercised in the producer suite.
    return pressure_cohort(source)+"\ndaq_calibration_t daq_calibration_current(void){return (daq_calibration_t){0};}\nstatic uint64_t g_sd_raw_key_floor;\n"+'\n'.join(function(source,n) for n in (
        'sd_card_capture_retain','sd_card_capture_begin','sd_raw_key_locked',
        'sd_card_enqueue_raw_adc_samples_captured_impl',
        'sd_card_enqueue_raw_adc_samples_captured','sd_card_enqueue_raw_adc_samples'))+'\n'


def route_epoch_support(source):
    begin=source.index('static struct { uint32_t current, old, cutoff, flags;')
    end=source.index('static void sd_route_epoch_advance(',begin)
    return source[begin:end]+'\n'+function(source,'sd_route_epoch_advance')+'\n'


def ordinary_owner_fixture(code, source):
    # Source-derived refcount/route dependencies for pre-existing ordinary row
    # tests. No seal or thermal activation is exercised or claimed here.
    import re
    begin=source.index('static struct { uint32_t current, old, cutoff, flags;')
    end=source.index('static void sd_route_epoch_advance(',begin)
    support=source[begin:end]+"\ndaq_calibration_t daq_calibration_current(void){daq_calibration_t c;(void)sd_calibration_snapshot(&c);return c;}\n"
    support+='\n'.join(function(source,n) for n in ('sd_route_epoch_advance',
        'sd_capture_old_locked','sd_capture_retain_locked','sd_card_capture_retain',
        'sd_card_capture_release','sd_card_capture_fault','sd_card_capture_begin'))+'\n'
    # Existing fixture's RAM-only calibration collaborator stays unchanged.
    marker='static unsigned sd_calibration_snapshot(daq_calibration_t *c){memset(c,0,sizeof(*c));c->kg1000_slope=1;return 1;}'
    assert code.count(marker)==1
    code=code.replace(marker,marker+'\n'+support)
    code=code.replace('/* SOURCE_FUNCTION:sd_route_session_locked */',
        'static void sd_route_epoch_advance(void);\n/* SOURCE_FUNCTION:sd_route_session_locked */')
    for legacy in ('sd_card_enqueue_csv_row','sd_card_enqueue_raw_adc_samples','sd_card_enqueue_optional_snapshot'):
        marker='/* SOURCE_FUNCTION:'+legacy+' */'
        if marker not in code: continue
        dependencies='\n'.join(function(source,n) for n in (legacy+'_captured_impl',legacy+'_captured'))
        code=code.replace(marker,dependencies+'\n'+marker)
    return code
